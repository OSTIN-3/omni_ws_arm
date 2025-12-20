import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time
import threading

# =================================================================
# [사용자 설정] 최종 목적지 좌표 (RViz에서 확인 후 수정하세요)
# =================================================================
# 1. 시작점은 현재 로봇 위치 (좌표 필요 없음)
# 2. 도착점 (Goal Pose)
GOAL_COORDS = {'x': 2.0, 'y': -0.5, 'w': 1.0} 
# =================================================================

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        # 통신 설정
        self.pub_agv_status = self.create_publisher(String, '/agv/status', 10)
        self.sub_arm_status = self.create_subscription(
            String, '/arm/status', self.arm_callback, 10)
        
        self.latest_arm_status = None 
        self.get_logger().info("✅ Mission Controller Logic Ready!")

    def arm_callback(self, msg):
        self.latest_arm_status = msg.data
        self.get_logger().info(f"📩 Arm Status: {msg.data}")

    def send_command(self, cmd):
        """로봇팔에게 명령 전송 후 응답 초기화"""
        self.latest_arm_status = None # 이전 상태 지우기 (중요)
        msg = String()
        msg.data = cmd
        self.pub_agv_status.publish(msg)
        self.get_logger().info(f"📤 Command to Arm: {cmd}")

def create_pose(navigator, coords):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = coords['x']
    pose.pose.position.y = coords['y']
    pose.pose.orientation.w = coords['w']
    pose.pose.orientation.z = 0.0 
    return pose

def main(args=None):
    rclpy.init(args=args)

    # 노드 초기화
    mission_node = MissionNode()
    navigator = BasicNavigator()

    # 통신 수신을 위한 백그라운드 스레드
    spinner_thread = threading.Thread(target=rclpy.spin, args=(mission_node,), daemon=True)
    spinner_thread.start()

    # Nav2 활성화 대기
    print("⏳ Waiting for Nav2...")
    navigator.waitUntilNav2Active()
    print("✅ System Ready. Mission Start!")

    # =============================================================
    # [STEP 1] 시작 지점 작업 (집기 + 무한 재시도)
    # =============================================================
    print("🏁 [1단계] 시작 지점 대기 (2초)...")
    time.sleep(2.0) # 사용자가 요청한 2초 대기 (Home Pose 대기 효과)

    print("🤖 [1단계] 물체 집기 시도...")
    
    while True:
        # 1. 집기 명령 전송
        mission_node.send_command("ARRIVED_PICK")
        
        # 2. 응답 대기 (GRIPPED or GRIPPED_FAIL)
        print("⏳ 로봇팔 작업 중...")
        while mission_node.latest_arm_status is None:
            time.sleep(0.1)
        
        # 3. 결과 확인
        result = mission_node.latest_arm_status
        
        if result == "GRIPPED":
            print("🎉 성공: 물체를 잡았습니다! 이동 준비.")
            break # 반복문 탈출 -> 이동 시작
            
        elif result == "GRIPPED_FAIL":
            print("⚠️ 실패: 물체를 놓쳤습니다. 3초 후 재시도합니다...")
            time.sleep(3.0) # 재시도 전 대기
            # loop 다시 시작 (send_command 부터)
            
        else:
            # 혹시 모를 다른 메시지 처리
            time.sleep(1.0)

    # =============================================================
    # [STEP 2] 네비게이션 (Goal Pose로 이동)
    # =============================================================
    goal_pose = create_pose(navigator, GOAL_COORDS)
    print(f"🚀 [2단계] 목표 지점으로 이동 중... (x={GOAL_COORDS['x']})")
    
    navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete():
        # 이동 중 피드백 출력 (선택사항)
        pass

    # 이동 결과 확인
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("✅ 도착: 목표 지점 도착 완료!")
    else:
        print(f"❌ 이동 실패! (Code: {result})")
        # 실패해도 일단 놓기 시도는 할지, 종료할지 결정. 여기선 종료.
        return 

    # =============================================================
    # [STEP 3] 도착 지점 작업 (놓기)
    # =============================================================
    print("🤖 [3단계] 물체 놓기 작업 시작...")
    mission_node.send_command("ARRIVED_PLACE")

    # 완료 대기 (RELEASED)
    while mission_node.latest_arm_status != "RELEASED":
        time.sleep(0.5)

    print("🎉 성공: 물체 놓기 완료.")
    print("🏁 [미션 종료] 수고하셨습니다!")

    mission_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
