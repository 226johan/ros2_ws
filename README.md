sudo apt install espeak

sudo apt install libttspico-utils
## 键盘控制节点 
* ros2 run wpr_simulation keyboard_vel_cmd

## 保存地图
* ros2 run nav2_map_server map_saver_cli -f map

## 制作航点
* ros2 launch wp_map_tools add_waypoints_sim.launch.py

## 保存航点
* ros2 run wp_map_tools wp_saver

## 启动机器人仿真环境
* ros2 luanch wpr_simulation robotcup_home.launch.py

## 初始化位姿
* ros2 run initpose_pkg initpose

## 航点导航
* ros2 launch nav_pkg waypoint_nav.launch.py
* ros2 run nav_pkg waypoint_navigation


