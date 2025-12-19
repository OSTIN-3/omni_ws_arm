#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray, Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Twist 
from tf2_ros import TransformBroadcaster
import math
import serial
import threading
import re
import time

# --- 상수 정의 ---
DEG2RAD = math.pi / 180.0
G_TO_MS2 = 9.80665  # 1g = 9.80665 m/s^2

class OmniRun(Node):
    def __init__(self):
        super().__init__('omnirun')
        
        # 1. 시리얼 포트 설정
        try:
            self.serial_port = serial.Serial(
                port='/dev/ttyUSB1',   # 포트 확인 (/dev/ttyUSB0 or /dev/ttyUSB1)
                baudrate=115200,
                timeout=0.05
            )
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.serial_port = None
        
        # 2. 토픽 구독 (Subscriber)
        # (1) 수동 문자열 명령
        self.motor_cmd_sub = self.create_subscription(String, '/motor_cmd', self.motor_cmd_callback, 10)
        # (2) Nav2/Teleop 속도 명령 (핵심 기능)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # 3. 토픽 발행 (Publisher)
        self.encoder_pub = self.create_publisher(Int32MultiArray, '/encoder_data', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.speed_pub = self.create_publisher(Float32MultiArray, '/motor_speeds', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF Broadcaster (필요 시 주석 해제하여 사용)
        # self.tf_broadcaster = TransformBroadcaster(self)
        
        # 백그라운드 스레드 시작
        self.running = True
        if self.serial_port:
            self.serial_thread = threading.Thread(target=self.read_serial)
            self.serial_thread.daemon = True
            self.serial_thread.start()
        
        self.get_logger().info('OmniRun Node started (All Features & Logs Enabled)')
    
    # ==========================================================================
    # [기능 1] cmd_vel 콜백 (Nav2 명령 -> STM32 전송)
    # ==========================================================================
    def cmd_vel_callback(self, msg):
        if not self.serial_port or not self.serial_port.is_open:
            return
        
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z
        
        # 프로토콜: "V,vx,vy,omega\n"
        cmd = f"V,{vx:.3f},{vy:.3f},{omega:.3f}\n"
        
        try:
            self.serial_port.write(cmd.encode())
            # (선택) 명령 전송 로그는 너무 빠르므로 필요할 때만 주석 해제
            # self.get_logger().info(f'TX Vel: {cmd.strip()}')
        except Exception as e:
            self.get_logger().error(f"Serial write error (cmd_vel): {e}")

    # ==========================================================================
    # [기능 2] 문자열 명령 콜백
    # ==========================================================================
    def motor_cmd_callback(self, msg):
        if not self.serial_port: return
        cmd = msg.data
        if not cmd.endswith('\n'):
            cmd += '\n'
        try:
            self.serial_port.write(cmd.encode())
            self.get_logger().info(f'TX String: "{cmd.strip()}"')
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")
            
    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.x = 0.0; q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q
    
    # ==========================================================================
    # [기능 3] 시리얼 수신 및 데이터 파싱 (핵심 루프)
    # ==========================================================================
    def read_serial(self):
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode(errors='replace').strip()
                    if not line: continue

                    # ----------------------------------------------------------
                    # (A) STM32 시스템 메시지 처리 (색상 로그)
                    # ----------------------------------------------------------
                    if "[OK]" in line or "Success" in line:
                        self.get_logger().info(f"✅ STM32: {line}")
                    elif "[FAIL]" in line or "Error" in line or "Failed" in line:
                        self.get_logger().error(f"❌ STM32: {line}")
                    elif "Initializing" in line or "System" in line or "Calibrating" in line:
                        self.get_logger().warn(f"⚙️ STM32: {line}")

                    # ----------------------------------------------------------
                    # (B) 센서 데이터 파싱 및 로그 출력
                    # ----------------------------------------------------------
                    
                    # 1. Encoders
                    elif line.startswith("Encoders"):
                        pattern = r'Encoders - M1:(-?\d+) \| M2:(-?\d+) \| M3:(-?\d+)'
                        match = re.search(pattern, line)
                        if match:
                            msg = Int32MultiArray()
                            msg.data = [int(match.group(1)), int(match.group(2)), int(match.group(3))]
                            self.encoder_pub.publish(msg)
                            # [로그 활성화] 1초에 한 번씩 출력
                            self.get_logger().info(f"Enc: {msg.data}", throttle_duration_sec=1.0)
                    
                    # 2. IMU
                    elif line.startswith("IMU"):
                        pattern = r'IMU - Yaw:([+-]?\d+\.\d+) \| Gz:([+-]?\d+\.\d+) \| Ax:([+-]?\d+\.\d+) \| Ay:([+-]?\d+\.\d+)'
                        match = re.search(pattern, line)
                        if match:
                            yaw_deg = float(match.group(1))
                            gz_deg_s = float(match.group(2))
                            ax_g = float(match.group(3))
                            ay_g = float(match.group(4))
                            
                            # 좌표계 변환 (부호 반전 유지)
                            yaw_rad = -1.0 * yaw_deg * DEG2RAD
                            gz_rad_s = -1.0 * gz_deg_s * DEG2RAD
                            ax_ms2 = ax_g * G_TO_MS2
                            ay_ms2 = ay_g * G_TO_MS2
                            
                            imu_msg = Imu()
                            imu_msg.header.stamp = self.get_clock().now().to_msg()
                            imu_msg.header.frame_id = "imu_link"
                            imu_msg.orientation = self.yaw_to_quaternion(yaw_rad)
                            imu_msg.angular_velocity.z = gz_rad_s
                            imu_msg.linear_acceleration.x = ax_ms2
                            imu_msg.linear_acceleration.y = ay_ms2
                            imu_msg.linear_acceleration.z = 9.80665
                            
                            self.imu_pub.publish(imu_msg)
                            
                            # [로그 활성화] Yaw 값 확인용 로그
                            self.get_logger().info(f"IMU: Yaw={yaw_deg:.2f} deg", throttle_duration_sec=1.0)

                    # 3. Speed
                    elif line.startswith("Speed"):
                        pattern = r'M1:([+-]?\d+\.\d+) \| M2:([+-]?\d+\.\d+) \| M3:([+-]?\d+\.\d+)'
                        match = re.search(pattern, line)
                        if match:
                            msg = Float32MultiArray()
                            msg.data = [float(match.group(1)), float(match.group(2)), float(match.group(3))]
                            self.speed_pub.publish(msg)
                            # [로그 활성화]
                            self.get_logger().info(f"Speed: {msg.data}", throttle_duration_sec=1.0)
                            
                    # 4. Odom
                    elif line.startswith("Odom"):
                        pattern = r'Odom - x:([+-]?\d+\.\d+) \| y:([+-]?\d+\.\d+) \| th:([+-]?\d+\.\d+) \| vx:([+-]?\d+\.\d+) \| vy:([+-]?\d+\.\d+) \| w:([+-]?\d+\.\d+) \| t:(\d+)'
                        match = re.search(pattern, line)
                        if match:
                            x = float(match.group(1)); y = float(match.group(2)); theta = float(match.group(3))
                            vx = float(match.group(4)); vy = float(match.group(5)); omega = float(match.group(6))
                            
                            odom_msg = Odometry()
                            odom_msg.header.stamp = self.get_clock().now().to_msg()
                            odom_msg.header.frame_id = 'odom'
                            odom_msg.child_frame_id = 'base_link'
                            
                            odom_msg.pose.pose.position.x = x
                            odom_msg.pose.pose.position.y = y
                            odom_msg.pose.pose.orientation = self.yaw_to_quaternion(theta)
                            odom_msg.twist.twist.linear.x = vx
                            odom_msg.twist.twist.linear.y = vy
                            odom_msg.twist.twist.angular.z = omega
                            
                            self.odom_pub.publish(odom_msg)
                            
                            # [로그 활성화] 오도메트리 위치 로그
                            self.get_logger().info(f"Odom: x={x:.3f} y={y:.3f} th={theta:.3f}", throttle_duration_sec=1.0)
                    
                    # (C) 기타 알 수 없는 메시지
                    else:
                        # 너무 짧은 노이즈 데이터는 무시
                        if len(line) > 2:
                             # 알 수 없는 데이터도 일단 찍어줌 (디버깅용)
                             pass 
                             # self.get_logger().info(f"RAW: {line}")

            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")

    def destroy_node(self):
        self.running = False
        if self.serial_thread.is_alive():
            self.serial_thread.join()
        if self.serial_port:
            self.serial_port.close()
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