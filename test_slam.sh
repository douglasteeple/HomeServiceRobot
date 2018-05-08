#!/bin/bash
##################################################################
#
# Udacity Term 2 Home Service Project launch script
#          Douglas Teeple April 2018
#
# Test SLAM operation by using interactive markers or 
# teleop_keyboard to create a map.
#
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

echo -e ${BLUE}"Testing SLAM Project"${NC}
source /opt/ros/kinetic/setup.bash
source devel/setup.bash

project=home_service_project
ws=$HOME/catkin_ws/
world=U
world_file=${ws}/src/${project}/worlds/${world}.world
map_file=${ws}/src/${project}/maps/${world}.yaml
teleop=interactivemarkers

for arg in $*
do
	case ${arg} in
		teleop=*) teleop=${arg#teleop=};;
		*)	echo -e ${MAGENTA}"Unknown argument: ${arg}"${NC};
	 		echo -e ${GREEN}"Usage: $(basename $0): [teleop=keyboard|interactive]"${NC};
			;;
	esac
done


launch "roslaunch ${project} ${project}_world.launch world_file:=${world_file}"
launch "roslaunch ${project} gmapping.launch " 5

if [[ "${teleop}" == "keyboard" ]]
then
	launch "roslaunch turtlebot_teleop keyboard_teleop.launch" 5
else
	launch "roslaunch turtlebot_interactive_markers  interactive_markers.launch" 5
fi

