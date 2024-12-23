import os 
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    map_file=os.path.join(get_package_share_directory('nav_pkg'), 'maps', 'map.yaml')

    nav_param_file=os.path.join(get_package_share_directory('nav_pkg'), 'config', 'nav2_params.yaml')

    nav2_launch_dir=os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    navigation_cmd = IncludeLaunchDescription(PythonLaunchDescriptionSource([nav2_launch_dir, '/bringup_launch.py']),
                                                launch_arguments={'map': map_file, 
                                                                  'use_sim_time':use_sim_time,
                                                                  'params_file': nav_param_file}.items(),
                                                                  )
    rviz_file=os.path.join(get_package_share_directory('wp_map_tools'), 'rviz', 'navi.rviz')

    rviz_cmd = Node(package='rviz2', 
                    executable='rviz2', 
                    name='rviz2',
                    arguments=['-d', rviz_file],
                    )

    # 从主文件夹下的waypoints.yaml中获取保存的航点信息
    wp_edit_cmd = Node(package='wp_map_tools', 
                       executable='wp_edit_node', 
                       name='wp_edit_node',
                       )
    
    # 从话题/waterplus/navi_waypoints中获取目标航点名称，然后从wp_edif_node节点查询坐标和航向，
    # 然后调用nav2导航，导航完成后向话题/waterplus/navi_result发送信息navi done
    wp_navi_server_cmd = Node(package='wp_map_tools',
                              executable='wp_navi_server', 
                              name='wp_navi_server',
                              )
    
    ld=LaunchDescription()
    ld.add_action(navigation_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(wp_edit_cmd)
    ld.add_action(wp_navi_server_cmd)

    return ld