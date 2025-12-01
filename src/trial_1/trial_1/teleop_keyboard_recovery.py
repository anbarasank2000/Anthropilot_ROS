#!/usr/bin/env python3
"""
Teleop Keyboard with Recovery Integration

An enhanced keyboard teleoperation node that integrates with the teleop recovery system.
Includes special keys to trigger recovery mode and resume navigation.

Controls:
    Movement:
        w/x : increase/decrease linear velocity
        a/d : increase/decrease angular velocity
        s   : stop
        
    Arrow keys:
        UP    : move forward
        DOWN  : move backward
        LEFT  : turn left
        RIGHT : turn right
        
    Recovery controls:
        f : Force fail / trigger teleop recovery
        r : Resume navigation
        c : Cancel navigation
        q : Quit

"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String, Bool
import sys
import termios
import tty
import select
import threading


class TeleopKeyboardRecovery(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_recovery')
        
        # Parameters
        self.declare_parameter('linear_vel', 0.2)
        self.declare_parameter('angular_vel', 0.5)
        self.declare_parameter('linear_step', 0.05)
        self.declare_parameter('angular_step', 0.1)
        
        self.linear_vel = self.get_parameter('linear_vel').value
        self.angular_vel = self.get_parameter('angular_vel').value
        self.linear_step = self.get_parameter('linear_step').value
        self.angular_step = self.get_parameter('angular_step').value
        
        self.max_linear = 0.5
        self.max_angular = 2.0
        
        # Current velocities
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/teleop_cmd_vel', 10)
        self.trigger_pub = self.create_publisher(Empty, '/trigger_teleop_recovery', 10)
        self.resume_pub = self.create_publisher(Empty, '/resume_navigation', 10)
        self.cancel_pub = self.create_publisher(Empty, '/cancel_navigation', 10)
        
        # Subscriber for state
        self.current_state = "UNKNOWN"
        self.teleop_active = False
        self.state_sub = self.create_subscription(
            String,
            '/teleop_recovery/state',
            self.state_callback,
            10
        )
        self.teleop_active_sub = self.create_subscription(
            Bool,
            '/teleop_recovery/teleop_active',
            self.teleop_active_callback,
            10
        )
        
        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.running = True
        
    def state_callback(self, msg):
        self.current_state = msg.data
        
    def teleop_active_callback(self, msg):
        self.teleop_active = msg.data
        
    def get_key(self, timeout=0.1):
        """Get a single keypress with timeout"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
            # Handle arrow keys (escape sequences)
            if key == '\x1b':
                key += sys.stdin.read(2)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
        
    def publish_velocity(self):
        """Publish current velocity"""
        msg = Twist()
        msg.linear.x = self.current_linear
        msg.angular.z = self.current_angular
        self.cmd_vel_pub.publish(msg)
        
    def stop(self):
        """Stop the robot"""
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.publish_velocity()
        
    def print_status(self):
        """Print current status"""
        status = f"\r[State: {self.current_state}] "
        status += f"[Teleop: {'ACTIVE' if self.teleop_active else 'INACTIVE'}] "
        status += f"Linear: {self.current_linear:.2f} Angular: {self.current_angular:.2f}    "
        print(status, end='', flush=True)
        
    def print_help(self):
        """Print help message"""
        help_msg = """
╔════════════════════════════════════════════════════════════════╗
║           Teleop Keyboard with Recovery Integration            ║
╠════════════════════════════════════════════════════════════════╣
║  Movement Controls:                                            ║
║    w : Increase linear velocity                                ║
║    x : Decrease linear velocity                                ║
║    a : Increase angular velocity (turn left)                   ║
║    d : Decrease angular velocity (turn right)                  ║
║    s : Stop                                                    ║
║                                                                ║
║  Arrow Keys:                                                   ║
║    ↑ : Move forward at current linear velocity                 ║
║    ↓ : Move backward at current linear velocity                ║
║    ← : Turn left at current angular velocity                   ║
║    → : Turn right at current angular velocity                  ║
║                                                                ║
║  Recovery Controls:                                            ║
║    f : Force FAIL - Trigger teleop recovery mode               ║
║    r : RESUME navigation to last goal                          ║
║    c : CANCEL current navigation                               ║
║                                                                ║
║  Other:                                                        ║
║    h : Show this help                                          ║
║    q : Quit                                                    ║
╚════════════════════════════════════════════════════════════════╝
"""
        print(help_msg)
        
    def run(self):
        """Main loop"""
        self.print_help()
        print("\nReady for input...\n")
        
        try:
            while self.running and rclpy.ok():
                # Process ROS callbacks
                rclpy.spin_once(self, timeout_sec=0.01)
                
                key = self.get_key()
                
                if key == 'q':
                    self.running = False
                    self.stop()
                    break
                    
                elif key == 'w':
                    self.current_linear = min(self.current_linear + self.linear_step, self.max_linear)
                    self.publish_velocity()
                    
                elif key == 'x':
                    self.current_linear = max(self.current_linear - self.linear_step, -self.max_linear)
                    self.publish_velocity()
                    
                elif key == 'a':
                    self.current_angular = min(self.current_angular + self.angular_step, self.max_angular)
                    self.publish_velocity()
                    
                elif key == 'd':
                    self.current_angular = max(self.current_angular - self.angular_step, -self.max_angular)
                    self.publish_velocity()
                    
                elif key == 's':
                    self.stop()
                    
                elif key == '\x1b[A':  # Up arrow
                    self.current_linear = self.linear_vel
                    self.publish_velocity()
                    
                elif key == '\x1b[B':  # Down arrow
                    self.current_linear = -self.linear_vel
                    self.publish_velocity()
                    
                elif key == '\x1b[C':  # Right arrow
                    self.current_angular = -self.angular_vel
                    self.publish_velocity()
                    
                elif key == '\x1b[D':  # Left arrow
                    self.current_angular = self.angular_vel
                    self.publish_velocity()
                    
                elif key == 'f':
                    print("\n>>> TRIGGERING TELEOP RECOVERY <<<")
                    self.trigger_pub.publish(Empty())
                    
                elif key == 'r':
                    print("\n>>> RESUMING NAVIGATION <<<")
                    self.resume_pub.publish(Empty())
                    
                elif key == 'c':
                    print("\n>>> CANCELLING NAVIGATION <<<")
                    self.cancel_pub.publish(Empty())
                    
                elif key == 'h':
                    self.print_help()
                    
                # Print status
                self.print_status()
                
        except Exception as e:
            print(f"\nError: {e}")
        finally:
            self.stop()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print("\nTeleop keyboard stopped.")


def main(args=None):
    rclpy.init(args=args)
    
    node = TeleopKeyboardRecovery()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
