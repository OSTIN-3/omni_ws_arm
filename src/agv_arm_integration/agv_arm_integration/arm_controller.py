import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import threading

# =========================================================
# [설정] 아두이노 포트 (환경에 맞게 수정하세요)
# =========================================================
SERIAL_PORT = '/dev/ttyUSB2' 
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
            self.get_logger().error(f'❌ Serial Connection Failed: {e}')
            exit()

        # 2. Publisher (로봇팔 상태 -> AGV)
        # 토픽: /arm/status
        # 메시지: "GRIPPED"(성공), "GRIPPED_FAIL"(실패), "RELEASED"(놓기 완료)
        self.publisher_ = self.create_publisher(String, '/arm/status', 10)

        # 3. Subscriber (AGV 상태 -> 로봇팔)
        # 토픽: /agv/status
        # 메시지: "ARRIVED_PICK"(집기 시작), "ARRIVED_PLACE"(놓기 시작)
        self.subscription = self.create_subscription(
            String,
            '/agv/status',
            self.listener_callback,
            10)
        
        # 4. 시리얼 수신 스레드 시작
        self.running = True
        self.serial_thread = threading.Thread(target=self.serial_reader)
        self.serial_thread.daemon = True
        self.serial_thread.start()

        self.get_logger().info('🤖 Arm Controller Node is Ready! (Smart Detection Enabled)')

    def listener_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'📩 Received from AGV: "{command}"')

        if command == "ARRIVED_PICK":
            self.get_logger().info('🚀 Starting PICK Sequence...')
            self.send_serial("SEQ:PICK")
            
        elif command == "ARRIVED_PLACE":
            self.get_logger().info('🚀 Starting RELEASE Sequence...')
            self.send_serial("SEQ:RELEASE")
            
        else:
            self.get_logger().warn(f'Unknown command: {command}')

    def send_serial(self, cmd):
        if self.ser and self.ser.is_open:
            self.ser.write((cmd + '\n').encode())

    def serial_reader(self):
        """아두이노로부터 완료/실패 신호를 기다림"""
        while self.running:
            if self.ser and self.ser.in_waiting:
                try:
                    # [노이즈 방지] 깨진 데이터는 무시(ignore)하고 정상 데이터만 처리
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if not line:
                        continue

                    # 1. 집기 성공 (아두이노가 잡았다고 판단함)
                    if line == "DONE:PICK":
                        self.get_logger().info('✅ Pick Success! (Object Detected)')
                        msg = String()
                        msg.data = "GRIPPED"
                        self.publisher_.publish(msg)
                        self.get_logger().info(f'📤 Pub to AGV: "{msg.data}"')

                    # 2. [추가됨] 집기 실패 (허공을 잡음 -> 재시도 요청)
                    elif line == "FAIL:PICK":
                        self.get_logger().warn('⚠️ Pick Failed (Object not found)!')
                        msg = String()
                        msg.data = "GRIPPED_FAIL"
                        self.publisher_.publish(msg)
                        self.get_logger().info(f'📤 Pub to AGV: "{msg.data}" -> Retry Requested!')

                    # 3. 놓기 완료
                    elif line == "DONE:RELEASE":
                        self.get_logger().info('✅ Release Success!')
                        msg = String()
                        msg.data = "RELEASED"
                        self.publisher_.publish(msg)
                        self.get_logger().info(f'📤 Pub to AGV: "{msg.data}"')
                        
                    # 기타 아두이노 디버깅 메시지 출력 (선택 사항)
                    else:
                        # self.get_logger().info(f'[Arduino] {line}')
                        pass
                        
                except Exception as e:
                    self.get_logger().warn(f'Serial Read Warning: {e}')
            time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
