colcon build 
cmds=(  "sleep 2 && ros2 launch wpr_simulation2 robocup_home.launch.py"
	"sleep 3 && ros2 launch nav_pkg waypoint_nav.launch.py"
	"sleep 3 && ros2 run initpose_pkg initpose"
	"sleep 15 && ros2 run nav_pkg waypoint_navigation"
	)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done
