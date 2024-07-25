catkin_make
cmds=(
	"roscore"
	"roslaunch livox_ros_driver2 msg_MID360.launch"
	# "roslaunch linefit_ground_segmentation_ros segmentation.launch "
	"roslaunch  fast_lio mapping_mid360.launch"
	# "roslaunch pointcloud_to_laserscan sample_node.launch"
	"sleep 2 && roslaunch navigation navigation.launch"
	"roslaunch simple_robot robot.launch"
	)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source devel/setup.bash;$cmd;exec bash;"
	sleep 0.2
done