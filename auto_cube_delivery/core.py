# core.py
import rclpy

from modules.navigator import Navigator


def core_process():
    print("Starting Core Process...")

    navigator = Navigator()

    # x, y, yaw
    goal_pos = [0.0, 0.0, 0.0]
    navigation_is_done = navigator.set_goal(goal_pos[0], goal_pos[1], goal_pos[2], mode='degrees')
    if navigation_is_done:
        print("Navigation done")
    else:
        print("Fail to navigate. Shutting Down...")



if __name__ == '__main__':
    try:
        rclpy.init()
        core_process()
    except KeyboardInterrupt:
        print("Force Shutdown")
    finally:
        rclpy.shutdown()