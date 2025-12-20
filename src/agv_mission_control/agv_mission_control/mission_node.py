import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time
import threading

# =================================================================
# [사용자 설정] 맵 좌표 (Rviz2에서 '2D Goal Pose'로 확인 후 수정 필수!)
# =================================================================
# [x, y, z, w] (w는 방향/Quaternion의 w값, z는 회전각)
PICK_ZONE_COORDS  = {'x': 1.5, 'y': 0.5, 'w': 1.0} 
PLACE_ZONE_COORDS = {'x': 3.0, 'y': -1.0, 'w': 0.7}
FINAL_GOAL_COORDS = {'x': 0.0, 'y': 0.0, 'w': 1.0}
# =================================================================

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        # 1. 로봇팔과 통신 설정
        self.pub_agv_status = self.create_publisher(String, '/agv/status', 10)
        self.sub_arm_status = self.create_subscription(
            String, '/arm/status', self.arm_callback, 10)
        
        self.latest_arm_status = None # 로봇팔 응답 저장용
        self.get_logger().info("✅ Mission Controller Node Started!")

    def arm_callback(self, msg):
        """로봇팔이 보내는 신호 수신"""
        self.latest_arm_status = msg.data
        self.get_logger().info(f"📩 로봇팔 응답 수신: {msg.data}")

    def send_status(self, status_msg):
        """로봇팔에게 명령 전송"""
        msg = String()
        msg.data = status_msg
        self.pub_agv_status.publish(msg)
        self.get_logger().info(f"📤 로봇팔에게 전송: {status_msg}")
        self.latest_arm_status = None # 응답 초기화

def create_pose(navigator, coords):
    """좌표를 ROS2 Pose 메시지로 변환"""
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

    # 1. 노드 및 Nav2 초기화
    mission_node = MissionNode()
    navigator = BasicNavigator()

    # *중요* 통신 노드를 별도 스레드에서 돌려야 대기 중에도 메시지를 받음
    spinner_thread = threading.Thread(target=rclpy.spin, args=(mission_node,), daemon=True)
    spinner_thread.start()

    # 2. Nav2 활성화 대기
    print("⏳ Nav2 활성화 대기 중...")
    navigator.waitUntilNav2Active()
    print("✅ Nav2 준비 완료! 미션 시작!")

    # =============================================================
    # [STEP 1] 경유지 1 (Pick Zone) 이동
    # =============================================================
    pick_pose = create_pose(navigator, PICK_ZONE_COORDS)
    print("🚀 [1단계] 물체 집는 곳으로 이동...")
    navigator.goToPose(pick_pose)

    while not navigator.isTaskComplete():
        pass # 이동 중 대기

    if navigator.getResult() != TaskResult.SUCCEEDED:
        print("❌ 이동 실패! 미션 종료.")
        return

    # =============================================================
    # [STEP 2] 로봇팔 협동 (집기) - 재시도 로직 포함
    # =============================================================
    print("🔄 [2단계] 도착 완료. 로봇팔 작업 시작 (집기)")
    
    while True:
        mission_node.send_status("ARRIVED_PICK") # 도착 신호 발송
        
        # 응답 대기
        print("⏳ 로봇팔 작업 대기 중...")
        while mission_node.latest_arm_status is None:
            time.sleep(0.5)
        
        # 결과 확인
        status = mission_node.latest_arm_status
        if status == "GRIPPED":
            print("🎉 성공: 물체 집기 완료!")
            break 
        elif status == "GRIPPED_FAIL":
            print("⚠️ 실패: 잡기 실패. 3초 후 재시도...")
            time.sleep(3)
        else:
            time.sleep(1)

    # =============================================================
    # [STEP 3] 경유지 2 (Place Zone) 이동
    # =============================================================
    place_pose = create_pose(navigator, PLACE_ZONE_COORDS)
    print("🚀 [3단계] 물체 놓는 곳으로 이동...")
    navigator.goToPose(place_pose)

    while not navigator.isTaskComplete():
        pass

    if navigator.getResult() != TaskResult.SUCCEEDED:
        print("❌ 이동 실패!")
        return

    # =============================================================
    # [STEP 4] 로봇팔 협동 (놓기)
    # =============================================================
    print("🔄 [4단계] 도착 완료. 로봇팔 작업 시작 (놓기)")
    mission_node.send_status("ARRIVED_PLACE")
    
    while mission_node.latest_arm_status != "RELEASED":
        time.sleep(0.5)
        
    print("🎉 성공: 물체 놓기 완료!")

    # =============================================================
    # [STEP 5] 최종 복귀
    # =============================================================
    final_pose = create_pose(navigator, FINAL_GOAL_COORDS)
    print("🚀 [5단계] 복귀 지점으로 이동...")
    navigator.goToPose(final_pose)

    while not navigator.isTaskComplete():
        pass

    print("🏁 [미션 완료] 모든 작업 종료.")
    
    mission_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
