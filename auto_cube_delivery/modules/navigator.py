import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
import math
import time


class Navigator:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()

        self.navigator = BasicNavigator()


        # [중요 1] 초기 위치 설정 (이게 없으면 Nav2가 멍하니 기다릴 수 있음)
        # 실제 로봇/시뮬레이션의 시작 위치와 맞춰야 합니다.
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.w = 1.0

        # 이미 위치가 잡혀있지 않다면 초기화
        self.navigator.setInitialPose(initial_pose)

        print("Waiting Nav2...")
        self.navigator.waitUntilNav2Active()
        print("Done connecting Nav2. Ready to move.")

    def set_goal(self, x, y, yaw, mode='degrees'):
        goal_pose = PoseStamped()

        goal_pose.header.frame_id = 'map'   # world frame coordinate
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()

        # x, y in meter scale
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.position.z = 0.0

        if mode == 'degrees':
            yaw_rad = math.radians(yaw)
        elif mode == 'radians':
            yaw_rad = yaw
        else:
            assert False
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        print(f"Start Navigation: x={x}, y={y}, yaw={yaw}")
        self.navigator.goToPose(goal_pose)

        # # wait until done
        # while not self.navigator.isTaskComplete():
        #     # check status
        #     pass

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()

            # feedback이 None일 수도 있으므로 체크 필요
            if feedback:
                remain_dist = feedback.distance_remaining
                recoveries = feedback.number_of_recoveries

                # Duration 타입은 초(sec)로 변환해서 보는 게 편함
                elapsed_time = feedback.navigation_time.sec

                print(f'[이동중] 남은 거리: {remain_dist:.2f}m | '
                      f'경과 시간: {elapsed_time}초 | '
                      f'회복 시도: {recoveries}회')

            time.sleep(0.5) # 0.5초마다 로그 출력

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print("Done Navigation. Returning to Core Process...")
            return True
        else:
            print("Failed to Navigate. Returning to Core Process...")
            return False

    def stop(self):
        self.navigator.cancelTask()