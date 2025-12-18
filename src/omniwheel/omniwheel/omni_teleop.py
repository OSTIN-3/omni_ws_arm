#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
omni_teleop.py (English Version)
- Control Only (Teleop)
- Sends motor commands via Serial
- No sensor feedback reading
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class OmniTeleop(Node):
    def __init__(self):
        super().__init__('omni_teleop')
        
        # 1. Serial Port Setup
        # Check if your port is /dev/ttyUSB0 or /dev/ttyUSB1
        port_name = '/dev/ttyUSB1'
        baud_rate = 115200

        try:
            self.serial_port = serial.Serial(
                port=port_name,
                baudrate=baud_rate,
                timeout=0.05
            )
            self.get_logger().info(f'Connected to serial port: {port_name}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect serial ({port_name}): {e}')
            self.get_logger().error('Tip: Check permissions with "sudo chmod 666 /dev/ttyUSB1"')
            raise SystemExit
        
        # 2. Subscriber (Motor Command)
        # Subscribes to /motor_cmd and sends it to Serial
        self.motor_cmd_sub = self.create_subscription(
            String, 
            '/motor_cmd', 
            self.motor_cmd_callback, 
            10
        )
        
        self.get_logger().info('OmniTeleop Node Started (Control Only)')
    
    def motor_cmd_callback(self, msg):
        """Callback for sending teleop commands"""
        try:
            # Encode string to bytes and write to serial
            command = msg.data
            self.serial_port.write(command.encode('utf-8'))
            
            # Uncomment for debugging
            # self.get_logger().info(f'Sent: {command}')
            
        except Exception as e:
            self.get_logger().error(f'Serial Write Error: {e}')
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
        self.get_logger().info('Serial port closed. Node shutting down.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OmniTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
