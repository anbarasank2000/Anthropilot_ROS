#!/usr/bin/env python3
"""
Teleop Recovery Trigger CLI

A simple command-line tool to trigger teleop recovery mode during navigation.
This allows the operator to take control of the robot at any time.

Usage:
    ros2 run trial_1 trigger_teleop           # Trigger teleop mode
    ros2 run trial_1 trigger_teleop --resume  # Resume navigation
    ros2 run trial_1 trigger_teleop --cancel  # Cancel navigation
    ros2 run trial_1 trigger_teleop --status  # Get current status
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
import sys
import argparse


class TeleopTriggerNode(Node):
    def __init__(self):
        super().__init__('teleop_trigger')
        
        # Publishers
        self.trigger_pub = self.create_publisher(Empty, '/trigger_teleop_recovery', 10)
        self.resume_pub = self.create_publisher(Empty, '/resume_navigation', 10)
        self.cancel_pub = self.create_publisher(Empty, '/cancel_navigation', 10)
        
        # Subscriber for state
        self.current_state = None
        self.state_sub = self.create_subscription(
            String,
            '/teleop_recovery/state',
            self.state_callback,
            10
        )
        
    def state_callback(self, msg):
        self.current_state = msg.data
        
    def trigger_teleop(self):
        """Publish trigger to switch to teleop mode"""
        msg = Empty()
        self.trigger_pub.publish(msg)
        self.get_logger().info('Teleop recovery triggered!')
        
    def resume_navigation(self):
        """Publish resume command"""
        msg = Empty()
        self.resume_pub.publish(msg)
        self.get_logger().info('Resume navigation command sent!')
        
    def cancel_navigation(self):
        """Publish cancel command"""
        msg = Empty()
        self.cancel_pub.publish(msg)
        self.get_logger().info('Cancel navigation command sent!')
        
    def get_status(self):
        """Get current state"""
        # Spin briefly to receive state
        rclpy.spin_once(self, timeout_sec=1.0)
        if self.current_state:
            self.get_logger().info(f'Current state: {self.current_state}')
        else:
            self.get_logger().warn('Could not get current state - is teleop_recovery_node running?')


def main(args=None):
    parser = argparse.ArgumentParser(description='Teleop Recovery Trigger')
    parser.add_argument('--resume', '-r', action='store_true', help='Resume navigation')
    parser.add_argument('--cancel', '-c', action='store_true', help='Cancel navigation')
    parser.add_argument('--status', '-s', action='store_true', help='Get current status')
    
    # Parse known args to handle ROS args
    parsed_args, remaining = parser.parse_known_args()
    
    rclpy.init(args=remaining)
    node = TeleopTriggerNode()
    
    # Give time for publishers to connect
    import time
    time.sleep(0.5)
    
    try:
        if parsed_args.resume:
            node.resume_navigation()
        elif parsed_args.cancel:
            node.cancel_navigation()
        elif parsed_args.status:
            node.get_status()
        else:
            # Default: trigger teleop
            node.trigger_teleop()
            
        # Spin briefly to ensure message is sent
        rclpy.spin_once(node, timeout_sec=0.5)
        
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
