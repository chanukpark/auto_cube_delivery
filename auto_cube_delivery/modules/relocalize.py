import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32  # 에러 값을 받아올 메시지 타입 (예시)
import random
import time
import math

class AutoExplorer(Node):
    def __init__(self):
        super().__init__('auto_explorer')

        self.navigator = BasicNavigator()

        # [중요] SLAM 노드에서 Reprojection Error를 발행하는 토픽을 구독합니다.
        # 예: /orb_slam3/reprojection_error
        self.error_sub = self.create_subscription(
            Float32,
            '/reprojection_error',  # <-- 실제 토픽 이름으로 변경 필요
            self.error_callback,
            10
        )

        self.current_reproj_error = 999.0  # 초기값 (무한대)

        # 초기 위치 설정 (이전과 동일)
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(initial_pose)

        print("Waiting for Nav2...")
        self.navigator.waitUntilNav2Active()
        print("Nav2 Ready.")

    def error_callback(self, msg):
        """실시간으로 에러 값을 업데이트하는 콜백"""
        self.current_reproj_error = msg.data

    def get_random_pose(self, range_x=(-2.0, 2.0), range_y=(-2.0, 2.0)):
        """현재 위치 근처의 랜덤한 목표 지점 생성"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()

        # 랜덤 좌표 생성
        goal_pose.pose.position.x = random.uniform(range_x[0], range_x[1])
        goal_pose.pose.position.y = random.uniform(range_y[0], range_y[1])

        # 방향은 랜덤하게
        yaw = random.uniform(0, 2 * math.pi)
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

        return goal_pose

    def wander_until_converged(self, error_threshold=5.0):
        """
        [핵심 함수]
        에러가 threshold 이하로 떨어질 때까지 계속 랜덤 이동.
        """
        print(f"탐험 시작! 목표 Reprojection Error: {error_threshold} 이하")

        while rclpy.ok():
            # 1. 조건 만족 확인 (이동 시작 전 체크)
            if self.current_reproj_error < error_threshold:
                print(f"!!! 조건 만족 (초기 달성) !!! 현재 에러: {self.current_reproj_error}")
                return True

            # 2. 랜덤 목표 지점 생성
            goal = self.get_random_pose()
            print(f">> 새로운 랜덤 목표로 이동: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})")

            # 3. 이동 명령
            self.navigator.goToPose(goal)

            # 4. 이동 중 실시간 감시 (Monitor Loop)
            while not self.navigator.isTaskComplete():
                # [핵심] 이동 중이라도 에러가 낮아지면 즉시 정지!
                if self.current_reproj_error < error_threshold:
                    print(f"\n!!! 조건 만족 !!! 현재 에러: {self.current_reproj_error}")
                    print(">> 로봇 정지 명령 전송")
                    self.navigator.cancelTask() # 이동 취소
                    return True # 성공 리턴

                # 에러 로그 출력 (디버깅용)
                # print(f"이동 중... 현재 에러: {self.current_reproj_error:.2f}", end='\r')

                # ROS 콜백 처리를 위해 spin을 살짝 돌려줌 (데이터 갱신용)
                rclpy.spin_once(self, timeout_sec=0.1)

            # 5. 목표에 도착했거나 실패했을 경우
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print("\n목표 지점 도착. 아직 에러가 높음. 다음 장소로...")
            elif result == TaskResult.FAILED:
                print("\n이동 실패(갈 수 없는 곳). 다른 좌표 시도...")

            # 잠시 대기 후 반복
            time.sleep(0.5)

# ---------------------------------------------------------
# 실행 예시
# ---------------------------------------------------------
if __name__ == '__main__':
    rclpy.init()

    explorer = AutoExplorer()

    # 에러가 2.5 이하로 떨어질 때까지 돌아다녀라
    success = explorer.wander_until_converged(error_threshold=2.5)

    if success:
        print("최종 위치 수렴 완료!")

    rclpy.shutdown()




















    def localize_robot(self):
        """
        [핵심 기능]
        1. 글로벌 로컬라이제이션 서비스 호출 (파티클 뿌리기)
        2. 제자리 회전(Spin)을 통해 센서 데이터를 맵과 매칭
        3. 수렴된 현재 좌표 반환
        """
        print("\n--- [1단계] 글로벌 위치 추정 시작 ---")

        # 1. 서비스 클라이언트 생성 및 호출
        # /reinitialize_global_localization 서비스는 AMCL에게 "나 어디있는지 모르겠으니 다시 찾아봐"라고 명령함
        reinit_service = self.navigator.create_client(Empty, '/reinitialize_global_localization')

        if not reinit_service.wait_for_service(timeout_sec=3.0):
            print("서비스를 찾을 수 없습니다. AMCL이 실행 중인가요?")
            return False

        req = Empty.Request()
        future = reinit_service.call_async(req)
        rclpy.spin_until_future_complete(self.navigator, future)
        print(">> 파티클 전체 분산 완료. 위치 찾기를 시작합니다.")

        # 2. 위치 수렴을 위한 제자리 회전 (Spin)
        # 로봇이 돌아다니지 않고 그 자리에서 360도(6.28 rad) 돕니다.
        # 이게 "반경 내에서만 움직이는" 가장 안전한 방법입니다.
        print(">> 센서 매칭을 위해 제자리 회전 중...")
        self.navigator.spin(spin_dist=6.28, time_allowance=10) # 360도 회전

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            # print("회전 중...") # 필요하면 주석 해제
            time.sleep(0.5)

        # 회전 결과 확인
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(">> 회전 완료. 위치가 수렴되었습니다.")

            # 3. 찾은 위치 출력
            x, y, yaw = self.get_current_pose()
            if x is not None:
                print(f"!!! 찾은 현재 위치 !!! -> X: {x:.2f}, Y: {y:.2f}, Angle: {yaw:.1f}도")
                return True
            else:
                return False
        else:
            print("회전 실패! 위치를 찾지 못했습니다.")
            return False

