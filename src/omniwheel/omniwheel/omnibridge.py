#!/usr/bin/env python3
"""
omnibridge.py - STM32 ¼¾¼­ µ¥ÀÌÅÍ¸¦ ROS2 ÅäÇÈÀ¸·Î º¯È¯ÇÏ´Â Bridge ³ëµå

STM32·ÎºÎÅÍ ¹Þ´Â CSV Æ÷¸Ë:
DATA,timestamp,x,y,theta,vx,vy,omega,m1_rpm,m2_rpm,m3_rpm,yaw,gz

¹ßÇàÇÏ´Â ROS2 ÅäÇÈ:
- /odom (nav_msgs/Odometry): ·Îº¿ À§Ä¡¿Í ¼Óµµ
- /imu/data (sensor_msgs/Imu): IMU ¼¾¼­ µ¥ÀÌÅÍ
- /joint_states (sensor_msgs/JointState): ¸ðÅÍ RPM
- /tf (odom ¡æ base_link º¯È¯)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ROS2 ¸Þ½ÃÁö Å¸ÀÔ
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import TransformStamped, Quaternion
from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# TF2
from tf2_ros import TransformBroadcaster

# ½Ã¸®¾ó Åë½Å
import serial
import serial.tools.list_ports

# À¯Æ¿¸®Æ¼
import math
import time
import threading
from collections import deque


class OmniBridge(Node):
    """STM32¿Í ROS2¸¦ ¿¬°áÇÏ´Â Bridge ³ëµå"""
    
    def __init__(self):
        super().__init__('omnibridge')
        
        # ============================================
        # ÆÄ¶ó¹ÌÅÍ ¼±¾ð
        # ============================================
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        
        # ÆÄ¶ó¹ÌÅÍ °¡Á®¿À±â
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        # ============================================
        # QoS ¼³Á¤ (Åë½Å Ç°Áú)
        # ============================================
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # ============================================
        # Publishers »ý¼º
        # ============================================
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos_profile)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', qos_profile)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        
        # TF2 Broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # ============================================
        # ½Ã¸®¾ó Æ÷Æ® ÃÊ±âÈ­
        # ============================================
        self.serial_conn = None
        self.serial_lock = threading.Lock()
        self.connect_serial()
        
        # ============================================
        # »óÅÂ º¯¼ö
        # ============================================
        self.last_data_time = time.time()
        self.data_count = 0
        self.error_count = 0
        self.receive_rate = deque(maxlen=50)  # ÃÖ±Ù 50°³ µ¥ÀÌÅÍ ¼Óµµ ÃßÀû
        
        # ============================================
        # Å¸ÀÌ¸Ó »ý¼º
        # ============================================
        # ½Ã¸®¾ó ÀÐ±â (°¡´ÉÇÑ ºü¸£°Ô)
        self.create_timer(0.01, self.serial_read_callback)  # 100Hz
        
        # Áø´Ü Á¤º¸ ¹ßÇà (´À¸®°Ô)
        self.create_timer(1.0, self.publish_diagnostics)  # 1Hz
        
        self.get_logger().info('=== OmniBridge Node Started ===')
        self.get_logger().info(f'Serial Port: {self.serial_port}')
        self.get_logger().info(f'Baud Rate: {self.baud_rate}')
        self.get_logger().info(f'Frames: {self.odom_frame} -> {self.base_frame}')
    
    # ============================================
    # ½Ã¸®¾ó ¿¬°á
    # ============================================
    def connect_serial(self):
        """½Ã¸®¾ó Æ÷Æ® ¿¬°á"""
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.timeout,
                write_timeout=self.timeout
            )
            
            # ¹öÆÛ Å¬¸®¾î
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            
            self.get_logger().info(f'? Connected to {self.serial_port}')
            return True
            
        except serial.SerialException as e:
            self.get_logger().error(f'? Failed to connect: {e}')
            self.get_logger().warn('Available ports:')
            for port in serial.tools.list_ports.comports():
                self.get_logger().warn(f'  - {port.device}: {port.description}')
            return False
    
    # ============================================
    # ½Ã¸®¾ó µ¥ÀÌÅÍ ÀÐ±â (¸ÞÀÎ ÄÝ¹é)
    # ============================================
    def serial_read_callback(self):
        """½Ã¸®¾ó Æ÷Æ®¿¡¼­ µ¥ÀÌÅÍ ÀÐ±â ¹× Ã³¸®"""
        
        if not self.serial_conn or not self.serial_conn.is_open:
            # Àç¿¬°á ½Ãµµ
            if time.time() - self.last_data_time > 5.0:
                self.get_logger().warn('Attempting to reconnect...')
                self.connect_serial()
            return
        
        try:
            with self.serial_lock:
                if self.serial_conn.in_waiting > 0:
                    # ÇÑ ÁÙ ÀÐ±â
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    
                    # ºó ÁÙ ¹«½Ã
                    if not line:
                        return
                    
                    # µ¥ÀÌÅÍ Ã³¸®
                    self.process_line(line)
                    
        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')
            self.serial_conn.close()
            self.serial_conn = None
            
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')
            self.error_count += 1
    
    # ============================================
    # µ¥ÀÌÅÍ ÆÄ½Ì ¹× Ã³¸®
    # ============================================
    def process_line(self, line):
        """¼ö½ÅÇÑ ¶óÀÎ ÆÄ½Ì ¹× ROS2 ¸Þ½ÃÁö ¹ßÇà"""
        
        # DATA·Î ½ÃÀÛÇÏ´Â ÁÙ¸¸ Ã³¸®
        if not line.startswith('DATA,'):
            # µð¹ö±× ¸Þ½ÃÁö³ª ±âÅ¸ Á¤º¸´Â ·Î±×·Î¸¸ Ãâ·Â
            if line.strip():
                self.get_logger().debug(f'STM32: {line}')
            return
        
        try:
            # CSV ÆÄ½Ì
            # DATA,timestamp,x,y,theta,vx,vy,omega,m1_rpm,m2_rpm,m3_rpm,yaw,gz
            parts = line.split(',')
            
            if len(parts) != 13:
                self.get_logger().warn(f'Invalid data length: {len(parts)} (expected 13)')
                return
            
            # µ¥ÀÌÅÍ ÃßÃâ
            data = {
                'timestamp': int(parts[1]),      # ms
                'x': float(parts[2]),            # m
                'y': float(parts[3]),            # m
                'theta': float(parts[4]),        # rad
                'vx': float(parts[5]),           # m/s
                'vy': float(parts[6]),           # m/s
                'omega': float(parts[7]),        # rad/s
                'm1_rpm': float(parts[8]),       # rpm
                'm2_rpm': float(parts[9]),       # rpm
                'm3_rpm': float(parts[10]),      # rpm
                'yaw': float(parts[11]),         # deg
                'gz': float(parts[12]),          # deg/s
            }
            
            # ÇöÀç ROS ½Ã°£
            now = self.get_clock().now().to_msg()
            
            # °¢ ¸Þ½ÃÁö ¹ßÇà
            self.publish_odometry(data, now)
            self.publish_imu(data, now)
            self.publish_joint_states(data, now)
            
            # Åë°è ¾÷µ¥ÀÌÆ®
            self.data_count += 1
            current_time = time.time()
            dt = current_time - self.last_data_time
            if dt > 0:
                self.receive_rate.append(1.0 / dt)
            self.last_data_time = current_time
            
        except ValueError as e:
            self.get_logger().warn(f'Parse error: {e}')
            self.error_count += 1
            
        except Exception as e:
            self.get_logger().error(f'Processing error: {e}')
            self.error_count += 1
    
    # ============================================
    # Odometry ¸Þ½ÃÁö ¹ßÇà
    # ============================================
    def publish_odometry(self, data, timestamp):
        """Odometry ¸Þ½ÃÁö »ý¼º ¹× ¹ßÇà"""
        
        msg = Odometry()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        
        # À§Ä¡ (x, y, z=0)
        msg.pose.pose.position.x = data['x']
        msg.pose.pose.position.y = data['y']
        msg.pose.pose.position.z = 0.0
        
        # ¹æÇâ (theta ¡æ Quaternion)
        q = self.euler_to_quaternion(0, 0, data['theta'])
        msg.pose.pose.orientation = q
        
        # À§Ä¡ °øºÐ»ê (´ë°¢¼± Çà·Ä, 6x6)
        # [x, y, z, roll, pitch, yaw]
        msg.pose.covariance = [
            0.01, 0.0,  0.0, 0.0, 0.0, 0.0,  # x
            0.0,  0.01, 0.0, 0.0, 0.0, 0.0,  # y
            0.0,  0.0,  0.0, 0.0, 0.0, 0.0,  # z (»ç¿ë ¾È ÇÔ)
            0.0,  0.0,  0.0, 0.0, 0.0, 0.0,  # roll (»ç¿ë ¾È ÇÔ)
            0.0,  0.0,  0.0, 0.0, 0.0, 0.0,  # pitch (»ç¿ë ¾È ÇÔ)
            0.0,  0.0,  0.0, 0.0, 0.0, 0.05  # yaw
        ]
        
        # ¼Óµµ (·Îº¿ ÁÂÇ¥°è ±âÁØ)
        msg.twist.twist.linear.x = data['vx']
        msg.twist.twist.linear.y = data['vy']
        msg.twist.twist.linear.z = 0.0
        
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = data['omega']
        
        # ¼Óµµ °øºÐ»ê
        msg.twist.covariance = [
            0.02, 0.0,  0.0, 0.0, 0.0, 0.0,  # vx
            0.0,  0.02, 0.0, 0.0, 0.0, 0.0,  # vy
            0.0,  0.0,  0.0, 0.0, 0.0, 0.0,  # vz
            0.0,  0.0,  0.0, 0.0, 0.0, 0.0,  # wx
            0.0,  0.0,  0.0, 0.0, 0.0, 0.0,  # wy
            0.0,  0.0,  0.0, 0.0, 0.0, 0.03  # wz
        ]
        
        self.odom_pub.publish(msg)
        
        # TF ¹ßÇà
        if self.publish_tf:
            self.publish_transform(data, timestamp)
    
    # ============================================
    # TF º¯È¯ ¹ßÇà
    # ============================================
    def publish_transform(self, data, timestamp):
        """odom ¡æ base_link TF ¹ßÇà"""
        
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        # À§Ä¡
        t.transform.translation.x = data['x']
        t.transform.translation.y = data['y']
        t.transform.translation.z = 0.0
        
        # ¹æÇâ
        q = self.euler_to_quaternion(0, 0, data['theta'])
        t.transform.rotation = q
        
        self.tf_broadcaster.sendTransform(t)
    
    # ============================================
    # IMU ¸Þ½ÃÁö ¹ßÇà
    # ============================================
    def publish_imu(self, data, timestamp):
        """IMU ¸Þ½ÃÁö »ý¼º ¹× ¹ßÇà"""
        
        msg = Imu()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.base_frame
        
        # ¹æÇâ (Yaw¸¸ »ç¿ë, deg ¡æ rad º¯È¯)
        yaw_rad = data['yaw'] * math.pi / 180.0
        q = self.euler_to_quaternion(0, 0, yaw_rad)
        msg.orientation = q
        
        # ¹æÇâ °øºÐ»ê (Roll, Pitch´Â ÃøÁ¤ ¾È ÇÔ)
        msg.orientation_covariance = [
            -1.0, 0.0, 0.0,  # Roll (»ç¿ë ¾È ÇÔ)
            0.0, -1.0, 0.0,  # Pitch (»ç¿ë ¾È ÇÔ)
            0.0, 0.0, 0.05   # Yaw
        ]
        
        # °¢¼Óµµ (ZÃà¸¸, deg/s ¡æ rad/s)
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = data['gz'] * math.pi / 180.0
        
        msg.angular_velocity_covariance = [
            -1.0, 0.0, 0.0,  # X (»ç¿ë ¾È ÇÔ)
            0.0, -1.0, 0.0,  # Y (»ç¿ë ¾È ÇÔ)
            0.0, 0.0, 0.02   # Z
        ]
        
        # ¼±°¡¼Óµµ (ÃøÁ¤ ¾È ÇÔ)
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 0.0
        msg.linear_acceleration_covariance[0] = -1.0  # »ç¿ë ¾È ÇÔ Ç¥½Ã
        
        self.imu_pub.publish(msg)
    
    # ============================================
    # Joint States ¸Þ½ÃÁö ¹ßÇà
    # ============================================
    def publish_joint_states(self, data, timestamp):
        """¸ðÅÍ »óÅÂ (RPM) ¹ßÇà"""
        
        msg = JointState()
        msg.header.stamp = timestamp
        
        # ¸ðÅÍ ÀÌ¸§
        msg.name = ['motor1', 'motor2', 'motor3']
        
        # RPM °ª (rad/s·Î º¯È¯)
        rpm_to_rads = 2.0 * math.pi / 60.0
        msg.velocity = [
            data['m1_rpm'] * rpm_to_rads,
            data['m2_rpm'] * rpm_to_rads,
            data['m3_rpm'] * rpm_to_rads
        ]
        
        # À§Ä¡¿Í ÅäÅ©´Â ÃøÁ¤ ¾È ÇÔ
        msg.position = [0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0]
        
        self.joint_pub.publish(msg)
    
    # ============================================
    # Áø´Ü Á¤º¸ ¹ßÇà
    # ============================================
    def publish_diagnostics(self):
        """½Ã½ºÅÛ »óÅÂ Áø´Ü Á¤º¸ ¹ßÇà"""
        
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        status = DiagnosticStatus()
        status.name = 'OmniBridge: STM32 Connection'
        status.hardware_id = self.serial_port
        
        # ¿¬°á »óÅÂ È®ÀÎ
        time_since_data = time.time() - self.last_data_time
        
        if not self.serial_conn or not self.serial_conn.is_open:
            status.level = DiagnosticStatus.ERROR
            status.message = 'Serial port not connected'
        elif time_since_data > 1.0:
            status.level = DiagnosticStatus.WARN
            status.message = f'No data for {time_since_data:.1f}s'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'Receiving data'
        
        # Åë°è Á¤º¸
        avg_rate = sum(self.receive_rate) / len(self.receive_rate) if self.receive_rate else 0.0
        
        status.values = [
            KeyValue(key='Data Count', value=str(self.data_count)),
            KeyValue(key='Error Count', value=str(self.error_count)),
            KeyValue(key='Receive Rate', value=f'{avg_rate:.1f} Hz'),
            KeyValue(key='Time Since Data', value=f'{time_since_data:.2f}s'),
        ]
        
        msg.status.append(status)
        self.diag_pub.publish(msg)
    
    # ============================================
    # À¯Æ¿¸®Æ¼ ÇÔ¼ö
    # ============================================
    def euler_to_quaternion(self, roll, pitch, yaw):
        """¿ÀÀÏ·¯ °¢µµ¸¦ QuaternionÀ¸·Î º¯È¯"""
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        
        return q
    
    # ============================================
    # Á¾·á Ã³¸®
    # ============================================
    def destroy_node(self):
        """³ëµå Á¾·á ½Ã ½Ã¸®¾ó Æ÷Æ® ´Ý±â"""
        
        if self.serial_conn and self.serial_conn.is_open:
            self.get_logger().info('Closing serial port...')
            self.serial_conn.close()
        
        super().destroy_node()


# ============================================
# ¸ÞÀÎ ÇÔ¼ö
# ============================================
def main(args=None):
    rclpy.init(args=args)
    
    node = OmniBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
