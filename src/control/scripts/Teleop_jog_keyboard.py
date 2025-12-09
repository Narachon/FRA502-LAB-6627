#!/usr/bin/env python3
"""
Teleop Jog Keyboard Node for LAB4
Controls end effector velocity via keyboard

*** ต้อง call service เปลี่ยนเป็น TO mode ก่อนใช้งาน! ***
ros2 service call /controller_server interface/srv/ControllerData "{mode: {data: 'TO'}, position: {x: 0.0, y: 0.0, z: 0.0}}"

Controls:
---------
   w/s : +/- X velocity
   a/d : +/- Y velocity  
   q/e : +/- Z velocity
   +/- : Increase/Decrease speed
   Space : Stop
   Esc   : Quit
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

class TeleopJogKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_jog_keyboard')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.velocity = 0.1  # m/s
        
        self.get_logger().info("=== Teleop Jog Keyboard Started ===")
        self.print_instructions()
        
    def print_instructions(self):
        msg = """
╔═══════════════════════════════════════════════════════════╗
║           Teleop Jog Keyboard Control                     ║
╠═══════════════════════════════════════════════════════════╣
║  ⚠️  ต้องเปลี่ยนเป็น TO mode ก่อนใช้งาน!                    ║
║                                                           ║
║  World Frame:                                             ║
║  ros2 service call /controller_server                     ║
║    interface/srv/ControllerData                           ║
║    "{mode: {data: 'TO'}, position: {x: 0.0, ...}}"        ║
║                                                           ║
║  End Effector Frame:                                      ║
║  ros2 service call /controller_server                     ║
║    interface/srv/ControllerData                           ║
║    "{mode: {data: 'TO'}, position: {x: 1.0, ...}}"        ║
╠═══════════════════════════════════════════════════════════╣
║  Movement:                                                ║
║     w/s : +/- X velocity                                  ║
║     a/d : +/- Y velocity                                  ║
║     q/e : +/- Z velocity                                  ║
║                                                           ║
║  Settings:                                                ║
║     +/- : Increase/Decrease speed                         ║
║                                                           ║
║  Control:                                                 ║
║     Space : Stop all movement                             ║
║     Esc   : Quit                                          ║
╚═══════════════════════════════════════════════════════════╝
"""
        print(msg)
        
    def get_key(self):
        """Get keyboard input without blocking"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
    
    def run(self):
        """Main loop - Only publishes /cmd_vel, does NOT change mode"""
        print("\n🎮 Teleop ready! Publishing to /cmd_vel...")
        print("   (Make sure controller is in TO mode)\n")
        
        twist = Twist()
        
        try:
            while rclpy.ok():
                key = self.get_key()
                
                # Reset velocities
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                
                if key == 'w':
                    twist.linear.x = self.velocity
                    print(f"\r→ Moving +X ({self.velocity:.2f} m/s)         ", end='', flush=True)
                elif key == 's':
                    twist.linear.x = -self.velocity
                    print(f"\r→ Moving -X ({self.velocity:.2f} m/s)         ", end='', flush=True)
                elif key == 'a':
                    twist.linear.y = self.velocity
                    print(f"\r→ Moving +Y ({self.velocity:.2f} m/s)         ", end='', flush=True)
                elif key == 'd':
                    twist.linear.y = -self.velocity
                    print(f"\r→ Moving -Y ({self.velocity:.2f} m/s)         ", end='', flush=True)
                elif key == 'q':
                    twist.linear.z = self.velocity
                    print(f"\r→ Moving +Z ({self.velocity:.2f} m/s)         ", end='', flush=True)
                elif key == 'e':
                    twist.linear.z = -self.velocity
                    print(f"\r→ Moving -Z ({self.velocity:.2f} m/s)         ", end='', flush=True)
                elif key == ' ':
                    print("\r→ STOP                                  ", end='', flush=True)
                elif key == '+' or key == '=':
                    self.velocity = min(self.velocity + 0.05, 1.0)
                    print(f"\r→ Speed: {self.velocity:.2f} m/s              ", end='', flush=True)
                elif key == '-':
                    self.velocity = max(self.velocity - 0.05, 0.01)
                    print(f"\r→ Speed: {self.velocity:.2f} m/s              ", end='', flush=True)
                elif key == '\x1b':  # Escape
                    print("\n\rExiting...")
                    break
                
                # Publish velocity
                self.cmd_vel_pub.publish(twist)
                
        except KeyboardInterrupt:
            pass
        finally:
            # Stop robot
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            print("\n\rTeleop stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopJogKeyboard()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()