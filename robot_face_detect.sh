colcon build 
cmds=(  "ros2 launch wpr_simulation2 wpb_face.launch.py"
	"sleep 2 && ros2 run wpr_simulation2 face_detector.py"
	"sleep 3 && ros2 run cv_pkg cv_face_detect"
	"sleep 4 &&ros2 run wpr_simulation2 keyboard_vel_cmd"
	)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done
