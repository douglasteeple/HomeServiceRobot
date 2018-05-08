#!/bin/bash
##################################################################
#
# Udacity Term 2 Home Service Project launch script
#          Douglas Teeple April 2018
#
# Automate the process to let the robot follow the walls and 
# autonomously map the environment while avoiding obstacles. 
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

echo -e ${BLUE}"Starting WallFlower Project"${NC}
source /opt/ros/kinetic/setup.bash
source devel/setup.bash
 
source /opt/ros/kinetic/setup.bash
source devel/setup.bash
 
world=U
project=home_service_project
ws=$HOME/catkin_ws/
world_file=${ws}/src/${project}/worlds/${world}.world
mapname=${world}

for arg in $*
do
	case ${arg} in
		map=*)	mapname=${arg#map=};
			echo -e ${GREEN}"Saving map ${mapname}..."${NC};
			launch "rosrun map_server map_saver -f ${ws}/src/${project}/maps/${mapname}";
			exit;
			;;
		*)	echo -e ${MAGENTA}"Unknown argument: ${arg}"${NC};
	 		echo -e ${GREEN}"Usage: $(basename $0): [teleop=keyboard|interactive]"${NC};
			;;
	esac
done

launch "roslaunch ${project} ${project}_world.launch world_file:=${world_file}"
launch "roslaunch ${project} gmapping.launch " 5
launch "rosrun wall_follower wall_follower" 10
#launch "rosrun wall_follower wallFollowing"


