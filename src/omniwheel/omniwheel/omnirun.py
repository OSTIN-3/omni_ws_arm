#!/usr/bin/env python3
"""
omnirun.py (°³¼± ¹öÀü)
- ¸ðÅÍ Á¦¾î À¯Áö (teleop)
- Ç¥ÁØ IMU ¸Þ½ÃÁö »ç¿ë
- ¿¡·¯ Ã³¸® °­È­
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray, Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu  # ? Ç¥ÁØ IMU Ãß°¡!
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import math
import serial
import threading
import time

class OmniRun(Node):
    def __init__(self):
        super().__init__('omnirun')
        
        # ½Ã¸®¾ó Æ÷Æ®
        try:
            self.serial_port = serial.Serial(
                port='/dev/ttyUSB1',
                baudrate=115200,
                timeout=0.05
            )
            self.get_logger().info('½Ã¸®¾ó Æ÷Æ® ¿¬°á ¼º°ø')
        except serial.SerialException as e:
            self.get_logger().error(f'½Ã¸®¾ó ¿¬°á ½ÇÆÐ: {e}')
            raise SystemExit
        
        # ? Å¸ÀÓ¾Æ¿ô Ã¼Å©
        self.last_data_time = time.time()
        self.data_timeout = 5.0
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)  # ? Ç¥ÁØ!
        self.encoder_pub = self.create_publisher(Int32MultiArray, '/encoder_data', 10)
        self.speed_pub = self.create_publisher(Float32MultiArray, '/motor_speeds', 10)
        
        # Subscriber (¸ðÅÍ Á¦¾î)
        self.motor_cmd_sub = self.create_subscription(
            String, '/motor_cmd', self.motor_cmd_callback, 10
        )
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # ½Ã¸®¾ó ÀÐ±â ½º·¹µå
        self.running = True
        self.serial_thread = threading.Thread(target=self.read_serial)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        self.get_logger().info('OmniRun °³¼± ¹öÀü ½ÃÀÛ!')
    
    def motor_cmd_callback(self, msg):
        """Teleop ¸í·É Àü¼Û"""
        try:
            self.serial_port.write(msg.data.encode())
            self.get_logger().info(f'¸ðÅÍ ¸í·É: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'¸í·É Àü¼Û ½ÇÆÐ: {e}')
    
    def yaw_to_quaternion(self, yaw):
        """Yaw ¡æ Quaternion"""
        return Quaternion(
            x=0.0, y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0)
        )
    
    def read_serial(self):
        """½Ã¸®¾ó µ¥ÀÌÅÍ ÀÐ±â (½º·¹µå)"""
        import re
        
        while self.running:
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode(errors='ignore').strip()
                    
                    if not line:
                        # ? Å¸ÀÓ¾Æ¿ô Ã¼Å©
                        if (time.time() - self.last_data_time) > self.data_timeout:
                            self.get_logger().warn(
                                '5ÃÊ µ¿¾È µ¥ÀÌÅÍ ¾øÀ½!',
                                throttle_duration_sec=5.0
                            )
                        continue
                    
                    self.last_data_time = time.time()
                    
                    # Odometry
                    if line.startswith("Odom - x:"):
                        pattern = r'Odom - x:([+-]?\d+\.\d+) \| y:([+-]?\d+\.\d+) \| th:([+-]?\d+\.\d+) \| vx:([+-]?\d+\.\d+) \| vy:([+-]?\d+\.\d+) \| w:([+-]?\d+\.\d+)'
                        match = re.match(pattern, line)
                        if match:
                            x = float(match.group(1))
                            y = float(match.group(2))
                            theta = float(match.group(3))
                            vx = float(match.group(4))
                            vy = float(match.group(5))
                            omega = float(match.group(6))
                            
                            current_time = self.get_clock().now().to_msg()
                            quat = self.yaw_to_quaternion(theta)
                            
                            # Odometry ¹ßÇà
                            odom_msg = Odometry()
                            odom_msg.header.stamp = current_time
                            odom_msg.header.frame_id = 'odom'
                            odom_msg.child_frame_id = 'base_link'
                            odom_msg.pose.pose.position.x = x
                            odom_msg.pose.pose.position.y = y
                            odom_msg.pose.pose.orientation = quat
                            odom_msg.twist.twist.linear.x = vx
                            odom_msg.twist.twist.linear.y = vy
                            odom_msg.twist.twist.angular.z = omega
                            self.odom_pub.publish(odom_msg)
                            
                            # TF ¹ßÇà
                            t = TransformStamped()
                            t.header.stamp = current_time
                            t.header.frame_id = 'odom'
                            t.child_frame_id = 'base_link'
                            t.transform.translation.x = x
                            t.transform.translation.y = y
                            t.transform.translation.z = 0.0
                            t.transform.rotation = quat
                            self.tf_broadcaster.sendTransform(t)
                            
                            self.get_logger().info(
                                f"Odom: x={x:.3f} y={y:.3f} ¥è={theta:.3f}",
                                throttle_duration_sec=0.5
                            )
                    
                    # ? IMU (Ç¥ÁØ ¸Þ½ÃÁö!)
                    elif line.startswith("IMU - Yaw:"):
                        pattern = r'IMU - Yaw:([+-]?\d+\.\d+) \| Gz:([+-]?\d+\.\d+) \| Ax:([+-]?\d+\.\d+) \| Ay:([+-]?\d+\.\d+)'
                        match = re.match(pattern, line)
                        if match:
                            yaw_deg = float(match.group(1))
                            gz_deg = float(match.group(2))
                            ax_g = float(match.group(3))
                            ay_g = float(match.group(4))
                            
                            # ´ÜÀ§ º¯È¯
                            yaw_rad = math.radians(yaw_deg)
                            gz_rad = math.radians(gz_deg)
                            ax_ms2 = ax_g * 9.80665
                            ay_ms2 = ay_g * 9.80665
                            
                            # ? Ç¥ÁØ IMU ¸Þ½ÃÁö
                            imu_msg = Imu()
                            imu_msg.header.stamp = self.get_clock().now().to_msg()
                            imu_msg.header.frame_id = 'imu_link'
                            
                            # Orientation
                            imu_msg.orientation = self.yaw_to_quaternion(yaw_rad)
                            imu_msg.orientation_covariance[0] = 1e-6
                            imu_msg.orientation_covariance[4] = 1e-6
                            imu_msg.orientation_covariance[8] = 1e-6
                            
                            # Angular velocity
                            imu_msg.angular_velocity.z = gz_rad
                            
                            # Linear acceleration
                            imu_msg.linear_acceleration.x = ax_ms2
                            imu_msg.linear_acceleration.y = ay_ms2
                            
                            self.imu_pub.publish(imu_msg)
                            
                            self.get_logger().info(
                                f"IMU: Yaw={yaw_deg:.1f}¡Æ Gz={gz_deg:.1f}¡Æ/s",
                                throttle_duration_sec=0.5
                            )
                    
                    # Encoder
                    elif line.startswith("Encoders - M1:"):
                        pattern = r'Encoders - M1:(-?\d+) \| M2:(-?\d+) \| M3:(-?\d+)'
                        match = re.match(pattern, line)
                        if match:
                            msg = Int32MultiArray()
                            msg.data = [int(match.group(1)), int(match.group(2)), int(match.group(3))]
                            self.encoder_pub.publish(msg)
                    
                    # Speed
                    elif line.startswith("Speed - M1:"):
                        pattern = r'Speed - M1:([+-]?\d+\.\d+) \| M2:([+-]?\d+\.\d+) \| M3:([+-]?\d+\.\d+)'
                        match = re.match(pattern, line)
                        if match:
                            msg = Float32MultiArray()
                            msg.data = [float(match.group(1)), float(match.group(2)), float(match.group(3))]
                            self.speed_pub.publish(msg)
                    
            except serial.SerialException as e:
                self.get_logger().error(f'½Ã¸®¾ó ¿À·ù: {e}')
            except Exception as e:
                self.get_logger().error(f'Ã³¸® ¿À·ù: {e}')
    
    def destroy_node(self):
        """Á¾·á"""
        self.running = False
        self.serial_thread.join()
        self.serial_port.close()
        self.get_logger().info('½Ã¸®¾ó Á¾·á')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OmniRun()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
