#!/usr/bin/env python3
"""
Teleop Recovery Node

This node monitors navigation status and allows:
1. Automatic teleop takeover when navigation fails
2. Manual "fail" command to take over control during navigation
3. Seamless switching between autonomous navigation and teleoperation

Usage:
- Publish to /trigger_teleop_recovery (std_msgs/Empty) to manually trigger teleop mode
- Publish to /resume_navigation (std_msgs/Empty) to resume autonomous navigation
- The node automatically detects navigation failures and switches to teleop mode
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Empty, Bool, String
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from nav2_msgs.srv import ManageLifecycleNodes

import threading


class TeleopRecoveryNode(Node):
    """
    A node that provides teleop recovery functionality for Nav2 navigation.
    
    States:
    - IDLE: No navigation goal active
    - NAVIGATING: Robot is autonomously navigating to a goal
    - TELEOP_RECOVERY: User has taken over control via teleoperation
    - PAUSED: Navigation is paused, waiting for user input
    """
    
    # State constants
    STATE_IDLE = "IDLE"
    STATE_NAVIGATING = "NAVIGATING"
    STATE_TELEOP_RECOVERY = "TELEOP_RECOVERY"
    STATE_PAUSED = "PAUSED"
    
    def __init__(self):
        super().__init__('teleop_recovery_node')
        
        # Callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Current state
        self.current_state = self.STATE_IDLE
        self.state_lock = threading.Lock()
        
        # Store the last goal for potential retry
        self.last_goal = None
        self.current_goal_handle = None
        
        # Parameters
        self.declare_parameter('teleop_timeout', 30.0)  # Timeout in teleop mode before prompting
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('teleop_cmd_vel_topic', '/teleop_cmd_vel')
        self.declare_parameter('nav_cmd_vel_topic', '/nav_cmd_vel')
        self.declare_parameter('auto_resume_on_teleop_idle', False)
        self.declare_parameter('teleop_idle_timeout', 5.0)  # Seconds of no teleop input before auto-resume
        
        self.teleop_timeout = self.get_parameter('teleop_timeout').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.teleop_cmd_vel_topic = self.get_parameter('teleop_cmd_vel_topic').value
        self.nav_cmd_vel_topic = self.get_parameter('nav_cmd_vel_topic').value
        self.auto_resume = self.get_parameter('auto_resume_on_teleop_idle').value
        self.teleop_idle_timeout = self.get_parameter('teleop_idle_timeout').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(String, '/teleop_recovery/state', 10)
        self.teleop_active_pub = self.create_publisher(Bool, '/teleop_recovery/teleop_active', 10)
        
        # Subscribers
        self.teleop_cmd_vel_sub = self.create_subscription(
            Twist,
            self.teleop_cmd_vel_topic,
            self.teleop_cmd_vel_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.nav_cmd_vel_sub = self.create_subscription(
            Twist,
            self.nav_cmd_vel_topic,
            self.nav_cmd_vel_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Trigger teleop recovery manually
        self.trigger_teleop_sub = self.create_subscription(
            Empty,
            '/trigger_teleop_recovery',
            self.trigger_teleop_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Resume navigation
        self.resume_nav_sub = self.create_subscription(
            Empty,
            '/resume_navigation',
            self.resume_navigation_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Cancel current goal
        self.cancel_goal_sub = self.create_subscription(
            Empty,
            '/cancel_navigation',
            self.cancel_navigation_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Goal pose subscriber to intercept navigation goals
        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Nav2 action client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )
        
        # Timers
        self.state_timer = self.create_timer(0.5, self.publish_state)
        self.last_teleop_time = self.get_clock().now()
        self.teleop_idle_timer = None
        
        self.get_logger().info('Teleop Recovery Node initialized')
        self.get_logger().info(f'  - Teleop cmd_vel topic: {self.teleop_cmd_vel_topic}')
        self.get_logger().info(f'  - Nav cmd_vel topic: {self.nav_cmd_vel_topic}')
        self.get_logger().info(f'  - Output cmd_vel topic: {self.cmd_vel_topic}')
        self.get_logger().info('Commands:')
        self.get_logger().info('  - /trigger_teleop_recovery: Manually switch to teleop mode')
        self.get_logger().info('  - /resume_navigation: Resume autonomous navigation')
        self.get_logger().info('  - /cancel_navigation: Cancel current goal')
        
    def set_state(self, new_state):
        """Thread-safe state transition"""
        with self.state_lock:
            if self.current_state != new_state:
                self.get_logger().info(f'State transition: {self.current_state} -> {new_state}')
                self.current_state = new_state
                
    def get_state(self):
        """Thread-safe state getter"""
        with self.state_lock:
            return self.current_state
            
    def publish_state(self):
        """Publish current state periodically"""
        state_msg = String()
        state_msg.data = self.get_state()
        self.state_pub.publish(state_msg)
        
        teleop_active_msg = Bool()
        teleop_active_msg.data = (self.get_state() == self.STATE_TELEOP_RECOVERY)
        self.teleop_active_pub.publish(teleop_active_msg)
        
    def teleop_cmd_vel_callback(self, msg):
        """Handle teleop velocity commands"""
        current_state = self.get_state()
        
        # Update last teleop time
        self.last_teleop_time = self.get_clock().now()
        
        # If in teleop recovery mode, forward commands directly
        if current_state == self.STATE_TELEOP_RECOVERY:
            self.cmd_vel_pub.publish(msg)
        elif current_state == self.STATE_NAVIGATING:
            # If teleop command received during navigation, check if it's intentional
            # Non-zero commands indicate user wants to take over
            if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
                self.get_logger().warn('Teleop command received during navigation - switching to teleop mode')
                self.trigger_teleop_recovery()
                self.cmd_vel_pub.publish(msg)
        elif current_state == self.STATE_IDLE:
            # In idle state, just forward teleop commands
            self.cmd_vel_pub.publish(msg)
            
    def nav_cmd_vel_callback(self, msg):
        """Handle navigation velocity commands"""
        current_state = self.get_state()
        
        # Only forward nav commands if in navigating state
        if current_state == self.STATE_NAVIGATING:
            self.cmd_vel_pub.publish(msg)
            
    def trigger_teleop_callback(self, msg):
        """Handle manual teleop trigger"""
        self.get_logger().info('Manual teleop recovery triggered')
        self.trigger_teleop_recovery()
        
    def trigger_teleop_recovery(self):
        """Switch to teleop recovery mode"""
        current_state = self.get_state()
        
        if current_state == self.STATE_NAVIGATING:
            # Cancel current navigation goal
            if self.current_goal_handle is not None:
                self.get_logger().info('Cancelling current navigation goal...')
                cancel_future = self.current_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self.cancel_done_callback)
                
        # Stop the robot first
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        
        self.set_state(self.STATE_TELEOP_RECOVERY)
        self.get_logger().info('=== TELEOP RECOVERY MODE ACTIVE ===')
        self.get_logger().info('Use teleop to control the robot')
        self.get_logger().info('Publish to /resume_navigation to continue autonomous navigation')
        
        # Start idle timer if auto-resume is enabled
        if self.auto_resume:
            self.start_teleop_idle_timer()
            
    def cancel_done_callback(self, future):
        """Callback when goal cancellation is complete"""
        try:
            result = future.result()
            self.get_logger().info('Navigation goal cancelled successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to cancel goal: {e}')
            
    def resume_navigation_callback(self, msg):
        """Handle resume navigation command"""
        self.resume_navigation()
        
    def resume_navigation(self):
        """Resume autonomous navigation"""
        current_state = self.get_state()
        
        if current_state != self.STATE_TELEOP_RECOVERY and current_state != self.STATE_PAUSED:
            self.get_logger().warn(f'Cannot resume navigation from state: {current_state}')
            return
            
        # Stop any teleop idle timer
        if self.teleop_idle_timer is not None:
            self.teleop_idle_timer.cancel()
            self.teleop_idle_timer = None
            
        if self.last_goal is not None:
            self.get_logger().info('Resuming navigation to last goal...')
            self.send_goal(self.last_goal)
        else:
            self.get_logger().warn('No previous goal to resume. Setting state to IDLE.')
            self.set_state(self.STATE_IDLE)
            
    def cancel_navigation_callback(self, msg):
        """Handle cancel navigation command"""
        self.cancel_navigation()
        
    def cancel_navigation(self):
        """Cancel current navigation and go to idle"""
        if self.current_goal_handle is not None:
            self.get_logger().info('Cancelling navigation...')
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
            
        # Stop the robot
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        
        self.last_goal = None
        self.current_goal_handle = None
        self.set_state(self.STATE_IDLE)
        self.get_logger().info('Navigation cancelled. State: IDLE')
        
    def goal_pose_callback(self, msg):
        """Handle new goal pose"""
        self.get_logger().info(f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        self.last_goal = msg
        self.send_goal(msg)
        
    def send_goal(self, goal_pose):
        """Send navigation goal to Nav2"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available!')
            return
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.set_state(self.STATE_NAVIGATING)
        self.get_logger().info('Sending navigation goal...')
        
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        """Callback when goal is accepted/rejected"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected!')
            self.set_state(self.STATE_IDLE)
            return
            
        self.get_logger().info('Navigation goal accepted')
        self.current_goal_handle = goal_handle
        
        # Get result async
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
        
    def feedback_callback(self, feedback_msg):
        """Process navigation feedback"""
        feedback = feedback_msg.feedback
        # Could add progress logging here if needed
        pass
        
    def goal_result_callback(self, future):
        """Callback when navigation completes"""
        result = future.result()
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('=== NAVIGATION SUCCEEDED ===')
            self.set_state(self.STATE_IDLE)
            self.last_goal = None
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Navigation was cancelled')
            # Don't change state here - it's already set by the cancel function
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('=== NAVIGATION FAILED ===')
            self.get_logger().info('Switching to teleop recovery mode...')
            self.trigger_teleop_recovery()
        else:
            self.get_logger().warn(f'Navigation ended with status: {status}')
            self.set_state(self.STATE_IDLE)
            
        self.current_goal_handle = None
        
    def start_teleop_idle_timer(self):
        """Start timer to check for teleop idle"""
        if self.teleop_idle_timer is not None:
            self.teleop_idle_timer.cancel()
            
        self.teleop_idle_timer = self.create_timer(
            1.0,
            self.check_teleop_idle,
            callback_group=self.callback_group
        )
        
    def check_teleop_idle(self):
        """Check if teleop has been idle and auto-resume if enabled"""
        if self.get_state() != self.STATE_TELEOP_RECOVERY:
            if self.teleop_idle_timer is not None:
                self.teleop_idle_timer.cancel()
                self.teleop_idle_timer = None
            return
            
        time_since_teleop = (self.get_clock().now() - self.last_teleop_time).nanoseconds / 1e9
        
        if time_since_teleop > self.teleop_idle_timeout:
            self.get_logger().info(f'Teleop idle for {time_since_teleop:.1f}s - auto-resuming navigation')
            self.resume_navigation()


def main(args=None):
    rclpy.init(args=args)
    
    node = TeleopRecoveryNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
