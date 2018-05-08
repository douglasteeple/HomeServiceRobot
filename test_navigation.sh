#!/bin/bash
##################################################################
#
# Udacity Term 2 Home Service Project launch script
#          Douglas Teeple April 2018
#
# Manually point out to two different goals, one at a time, 
# and direct your robot to reach them and orient itself with 
# respect to them.
################################################################

. ./colors.sh	# enable terminal colors

function launch() {
	if [[ "$2" != "" ]]
	then
		sleep $2
	fi
	echo -e ${GREEN}"$1"${NC}
	xterm -e "$1" &
}

echo -e ${BLUE}"Starting Navigation Project"${NC}
echo -e ${BLUE}"Manually point out to two different 2D Pose goals,"${NC}
echo -e ${BLUE}"one at a time, and direct your robot to reach them"${NC}
echo -e ${BLUE}"and orient itself with respect to them."${NC}

project=home_service_project
ws=$HOME/catkin_ws/
world=U
world_file=${ws}/src/${project}/worlds/${world}.world
map_file=${ws}/src/${project}/maps/${world}.yaml

for arg in $*
do
	case ${arg} in
		world=*) world=${arg#world=};;
		*)	echo -e ${MAGENTA}"Unknown argument: ${arg}"${NC};
	 		echo -e ${GREEN}"Usage: $(basename $0): [teleop=keyboard|interactive]"${NC};
		;;
	esac
done

source /opt/ros/kinetic/setup.bash
source devel/setup.bash
 
launch "roslaunch ${project} ${project}_world.launch world_file:=${world_file}"
launch "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=${map_file} 3d_sensor:=kinect" 5
launch "roslaunch turtlebot_interactive_markers interactive_markers.launch" 5
#
# Some extras for logging and reports...
#
# to take a picture - right click on image
#
echo "rosrun image_view image_view image:=/camera/rgb/image_raw _filename_format:=${ws}/turtle%02i.jpg" 5
#
# to take a movie
#
echo "rosrun image_view video_recorder image:=/camera/rgb/image_raw" 5


