import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor 
from std_msgs.msg import String 
from geometry_msgs.msg import PoseStamped 
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
        
        # Teleop과 똑같이 String 'x' 명령을 보내기 위한 퍼블리셔
        self.pub_motor_cmd = self.create_publisher(String, '/motor_cmd', 10)
        
        # 2. 출발 신호 겸 목표 좌표 구독
        self.sub_start_signal = self.create_subscription(
            PoseStamped, '/mission/start', self.start_callback, 10)
        
        self.latest_arm_status = None 
        self.mission_start_flag = False 
        self.received_goal = None
        
        self.get_logger().info("✅ Mission Node Ready! (Waiting for Goal Pose...)")

    def arm_callback(self, msg):
        self.latest_arm_status = msg.data

    def start_callback(self, msg):
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        target_w = msg.pose.orientation.w
        
        self.received_goal = {'x': target_x, 'y': target_y, 'w': target_w}
        self.mission_start_flag = True
        self.get_logger().info(f"🚦 New Goal Received! x={target_x}, y={target_y}")

    def send_command(self, cmd):
        self.latest_arm_status = None 
        msg = String()
        msg.data = cmd
        self.pub_agv_status.publish(msg)
        self.get_logger().info(f"📤 Command to Arm: {cmd}")

    # =============================================================
    # [정지 함수] Teleop 'x' 키 기능
    # =============================================================
    def stop_robot(self):
        stop_msg = String()
        stop_msg.data = 'x'  # Teleop 코드에 있는 정지 문자
        
        # 3번 정도만 보내도 충분합니다. (너무 많이 보내면 딜레이 발생)
        for _ in range(3):
            self.pub_motor_cmd.publish(stop_msg)
            time.sleep(0.05) 
            
        self.get_logger().info("🛑 Robot Forced STOP (Sent 'x' to /motor_cmd)")

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
        print("👉 아래 명령어로 좌표를 보내면 출발합니다:")
        print("ros2 topic pub --once /mission/start geometry_msgs/msg/PoseStamped \"{pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}\"")
        
        # 🔥 [삭제됨] 대기 전에 stop_robot()을 호출하던 부분을 삭제했습니다.
        # 이제 시작 신호를 방해하지 않습니다.
        
        mission_node.mission_start_flag = False
        mission_node.received_goal = None
        
        # 플래그가 True가 될 때까지 순수 대기
        while not mission_node.mission_start_flag:
            time.sleep(0.5)

        current_goal_coords = mission_node.received_goal
        print(f"🚀 [START] 미션 시작! 목표: {current_goal_coords}")

        # [STEP 1] 집기
        # 출발 전 잠깐의 안정화 (필요 없으면 삭제 가능)
        time.sleep(1.0) 
        
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

        # [STEP 2] 네비게이션
        goal_pose = create_pose(navigator, current_goal_coords)
        print(f"🚀 [2단계] 목표 지점으로 이동... (x={current_goal_coords['x']})")
        
        navigator.goToPose(goal_pose)
        while not navigator.isTaskComplete():
            pass
        
        if navigator.getResult() != TaskResult.SUCCEEDED:
            print(f"❌ 이동 실패! 정지합니다.")
            mission_node.stop_robot() # 이동 실패 시에는 정지 필요
            continue

        # [STEP 3] 놓기
        print("🤖 [3단계] 물체 놓기...")
        mission_node.send_command("ARRIVED_PLACE")
        while mission_node.latest_arm_status != "RELEASED":
            time.sleep(0.5)
        print("🎉 성공: 놓기 완료.")

        # [STEP 4] 종료 처리
        print("🏁 [미션 완료] 로봇을 급정지('x') 합니다.")
        
        # 미션이 다 끝났을 때만 확실하게 정지 신호를 보냅니다.
        mission_node.stop_robot()
        
        time.sleep(1.0)
        print("🔄 ------------------------------------------ 🔄")

    navigator.lifecycleShutdown()
    mission_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()