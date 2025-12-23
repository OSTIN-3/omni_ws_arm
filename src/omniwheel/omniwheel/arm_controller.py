#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import threading

# =========================================================
# [설정] 아두이노 포트 확인 필수
# =========================================================
SERIAL_PORT = '/dev/ttyUSB0' 
BAUD_RATE = 115200

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # 1. 시리얼 연결
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            self.get_logger().info(f'✅ Serial Connected to {SERIAL_PORT}')
            time.sleep(2) # 아두이노 리셋 대기
        except Exception as e:
            self.get_logger().error(f'❌ Arm Serial Connection Failed: {e}')
            self.ser = None

        # 2. Publisher & Subscriber
        self.publisher_ = self.create_publisher(String, '/arm/status', 10)
        self.subscription = self.create_subscription(
            String, '/agv/status', self.listener_callback, 10)
        
        # 3. 시리얼 수신 스레드 시작
        self.running = True
        if self.ser:
            self.serial_thread = threading.Thread(target=self.serial_reader)
            self.serial_thread.daemon = True
            self.serial_thread.start()

        self.get_logger().info('🤖 Arm Controller Node Ready!')

    def listener_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'📩 Command from Mission: "{command}"')

        if command == "ARRIVED_PICK":
            self.send_serial("SEQ:PICK")
        elif command == "ARRIVED_PLACE":
            self.send_serial("SEQ:RELEASE")

    def send_serial(self, cmd):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((cmd + '\n').encode())
                self.get_logger().info(f'➡️ Send to Arduino: {cmd}')
            except Exception as e:
                self.get_logger().error(f"Serial write error: {e}")

    def serial_reader(self):
        """아두이노 응답 감지 및 디버그 로그 처리"""
        while self.running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line: continue

                    # [수정] 디버깅 로그 출력 (DEBUG: 또는 >>> 로 시작하는 메시지)
                    if line.startswith("DEBUG:") or line.startswith(">>>"):
                        print(f"[Arduino] {line}")
                        continue

                    # [1] 집기 성공
                    if line == "DONE:PICK":
                        self.get_logger().info('✅ Pick Success!')
                        self.publish_status("GRIPPED")

                    # [2] 집기 실패 (재시도 필요)
                    elif line == "FAIL:PICK":
                        self.get_logger().warn('⚠️ Pick Failed (Retrying...)')
                        self.publish_status("GRIPPED_FAIL")

                    # [3] 놓기 성공
                    elif line == "DONE:RELEASE":
                        self.get_logger().info('✅ Release Success!')
                        self.publish_status("RELEASED")
                        
            except Exception as e:
                self.get_logger().warn(f'Serial Read Warning: {e}')
                time.sleep(1)
            
            time.sleep(0.01)

    def publish_status(self, status_str):
        msg = String()
        msg.data = status_str
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
