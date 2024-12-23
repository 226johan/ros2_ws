colcon build 
cmds=(  "sleep 2 && ros2 launch wpr_simulation2 wpb_balls.launch.py"
	"sleep 3 && ros2 run cv_pkg cv_follow"
	"sleep 3 && ros2 run wpr_simulation2 ball_random_move"
	"sleep 3 && ros2 run wpr_simulation2 ball_random_move red_ball"
	"sleep 3 && ros2 run wpr_simulation2 ball_random_move blue_ball"
	"sleep 3 && ros2 run wpr_simulation2 ball_random_move green_ball"
	)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done
