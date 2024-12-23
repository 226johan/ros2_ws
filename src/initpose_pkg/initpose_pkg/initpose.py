from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from copy import deepcopy
from geometry_msgs.msg import PoseStamped


def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -5.6
    initial_pose.pose.position.y = -0.38
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    


if __name__ == '__main__':
    main()
