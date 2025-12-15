#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        # motor_cmd publisher (String)
        self.cmd_pub = self.create_publisher(String, '/motor_cmd', 10)
        self.print_instructions()

    def print_instructions(self):
        print("\n=== Omni-Wheel Robot Teleop ===")
        print("W/S: Forward/Backward")
        print("Q/E: Diagonal Up-Left/Up-Right")
        print("A/D: Diagonal Down-Left/Down-Right")
        print("R/T: Counter-Clockwise/Clockwise Rotation")
        print("F/G: Speed Increase/Decrease")
        print("P: PID Control Toggle")
        print("Y: Yaw Control Toggle")  # ? Ãß°¡!
        print("X/Space: Stop")
        print("Ctrl+C: Exit\n")

    def get_key(self):
        """Read keyboard input"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        """Main loop"""
        while rclpy.ok():
            key = self.get_key()

            # Ctrl+C exit
            if key == '\x03':
                break

            # Convert Spacebar to 'X' (Stop)
            if key == ' ':
                key = 'X'

            # Check if it is a valid command
            valid_keys = ['w', 'W', 's', 'S', 'q', 'Q', 'e', 'E', 
                          'a', 'A', 'd', 'D', 'r', 'R', 't', 'T', 
                          'f', 'F', 'g', 'G', 'x', 'X', 'p', 'P', 'y', 'Y']
            
            if key in valid_keys:
                # Publish as String message
                msg = String()
                msg.data = key
                self.cmd_pub.publish(msg)

                # Log output
                cmd_names = {
                    'w': 'Forward',
                    's': 'Backward',
                    'q': 'Diagonal Up-Left',
                    'e': 'Diagonal Up-Right',
                    'a': 'Diagonal Down-Left',
                    'd': 'Diagonal Down-Right',
                    'r': 'Counter-Clockwise Rotation',
                    't': 'Clockwise Rotation',
                    'f': 'Speed Increase',
                    'g': 'Speed Decrease',
                    'x': 'Stop',
                    'p': 'PID Toggle',
                    'y': 'Yaw Control Toggle'
                }
                
                key_lower = key.lower()
                if key_lower in cmd_names:
                    self.get_logger().info(f'{cmd_names[key_lower]} ({key})')

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command upon exit
        msg = String()
        msg.data = 'X'
        node.cmd_pub.publish(msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
