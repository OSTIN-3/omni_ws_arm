import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor 
from std_msgs.msg import String 
from geometry_msgs.msg import PoseStamped, Twist # [수정] Bool 삭제, PoseStamped 사용
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time
import threading

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        # 1. 통신 설정
        self.pub_agv_status = self.create_publisher(String, '/agv/status', 10)
        self.sub_arm_status = self.create_subscription(
            String, '/arm/status', self.arm_callback, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 2. [수정] 출발 신호 겸 목표 좌표 구독 (PoseStamped)
        # 이제 Bool(True/False)이 아니라 좌표(Pose)를 받으면 출발합니다.
        self.sub_start_signal = self.create_subscription(
            PoseStamped, '/mission/start', self.start_callback, 10)
        
        self.latest_arm_status = None 
        self.mission_start_flag = False 
        self.received_goal = None # [추가] 받은 좌표 저장용 변수
        
        self.get_logger().info("✅ Mission Node Ready! (Waiting for Goal Pose...)")

    def arm_callback(self, msg):
        self.latest_arm_status = msg.data

    # [수정] 콜백 함수가 이제 좌표를 함께 받습니다.
    def start_callback(self, msg):
        # 메시지에서 x, y, w 추출
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        target_w = msg.pose.orientation.w
        
        # 좌표 저장
        self.received_goal = {'x': target_x, 'y': target_y, 'w': target_w}
        
        # 출발 플래그 올리기
        self.mission_start_flag = True
        self.get_logger().info(f"🚦 New Goal Received! x={target_x}, y={target_y}")

    def send_command(self, cmd):
        self.latest_arm_status = None 
        msg = String()
        msg.data = cmd
        self.pub_agv_status.publish(msg)
        self.get_logger().info(f"📤 Command to Arm: {cmd}")

    def stop_robot(self):
        stop_msg = Twist()
        self.pub_cmd_vel.publish(stop_msg)
        self.get_logger().info("🛑 Robot Forced STOP")

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

    mission_node = MissionNode()
    navigator = BasicNavigator()

    executor = MultiThreadedExecutor()
    executor.add_node(mission_node)

    spinner_thread = threading.Thread(target=executor.spin, daemon=True)
    spinner_thread.start()

    print("⏳ Waiting for Nav2...")
    navigator.waitUntilNav2Active()
    print("✅ System Ready.")

    while rclpy.ok():
        # [STEP 0] 출발 대기 (좌표 수신 대기)
        print("\n🛑 [대기 중] 목표 좌표를 기다립니다...")
        print("👉 터미널에서 아래와 같이 좌표를 보내면 바로 시작합니다:")
        print("ros2 topic pub --once /mission/start geometry_msgs/msg/PoseStamped \"{pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}\"")
        
        mission_node.mission_start_flag = False
        mission_node.received_goal = None # 이전 좌표 초기화
        
        while not mission_node.mission_start_flag:
            time.sleep(0.5)

        # 수신된 좌표 가져오기
        current_goal_coords = mission_node.received_goal
        print(f"🚀 [START] 미션 시작! 목표: {current_goal_coords}")

        # [STEP 1] 집기
        print("🏁 [1단계] 시작 지점 대기 (2초)...")
        time.sleep(2.0)
        print("🤖 [1단계] 물체 집기 시도...")
        
        while True:
            mission_node.send_command("ARRIVED_PICK")
            while mission_node.latest_arm_status is None:
                time.sleep(0.1)
            result = mission_node.latest_arm_status
            if result == "GRIPPED":
                print("🎉 성공: 잡기 완료.")
                break 
            elif result == "GRIPPED_FAIL":
                print("⚠️ 실패: 재시도...")
                time.sleep(3.0)
            else:
                time.sleep(1.0)

        # [STEP 2] 네비게이션 (받은 좌표 사용!)
        goal_pose = create_pose(navigator, current_goal_coords)
        print(f"🚀 [2단계] 목표 지점으로 이동... (x={current_goal_coords['x']})")
        
        navigator.goToPose(goal_pose)
        while not navigator.isTaskComplete():
            pass
        
        if navigator.getResult() != TaskResult.SUCCEEDED:
            print(f"❌ 이동 실패! 다시 대기.")
            mission_node.stop_robot()
            continue

        # [STEP 3] 놓기
        print("🤖 [3단계] 물체 놓기...")
        mission_node.send_command("ARRIVED_PLACE")
        while mission_node.latest_arm_status != "RELEASED":
            time.sleep(0.5)
        print("🎉 성공: 놓기 완료.")

        # [STEP 4] 종료 처리
        print("🏁 [미션 완료] 정지 및 대기.")
        mission_node.stop_robot()
        time.sleep(1.0)
        print("🔄 ------------------------------------------ 🔄")

    navigator.lifecycleShutdown()
    mission_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()