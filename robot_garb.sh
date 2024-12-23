colcon build 
cmds=(  "sleep 2 && ros2 launch wpr_simulation2 wpb_table.launch.py"
	"sleep 3 && ros2 run wpr_simulation2 objects_publisher"
	"sleep 6 && ros2 run manipulator_pkg grab_object"
	)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done
