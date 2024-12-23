import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    slam_params={
        "use_sim_time":use_sim_time,
        "base_frame":"base_footprint",
        "odom_frame":"odom",
        "map_frame":"map"
    }

    slam_cmd=Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        parameters=[slam_params],
    )

    rviz_file=os.path.join(get_package_share_directory("slam_pkg"),"rviz","slam.rviz")

    rviz_cmd=Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d",rviz_file],
    )

    ld=LaunchDescription()
    ld.add_action(slam_cmd)
    ld.add_action(rviz_cmd)
    
    return ld
