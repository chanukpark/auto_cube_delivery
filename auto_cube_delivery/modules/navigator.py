import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import math

class Navigator:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()

        self.navigator = BasicNavigator()

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

        # wait until done
        while not self.navigator.isTaskComplete():
            # check status
            pass

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print("Done Navigation. Returning to Core Process...")
            return True
        else:
            print("Failed to Navigate. Returning to Core Process...")
            return False

    def stop(self):
        self.navigator.cancelTask()