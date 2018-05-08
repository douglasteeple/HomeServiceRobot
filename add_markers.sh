#!/bin/bash
##################################################################
#
# Udacity Term 2 Home Service Project launch script
#          Douglas Teeple April 2018
#
# Navigate to two goals and move a marker from the pickup goal to
# the dropoff goal.
# The first goal should be your desired pickup goal and the second goal
# should be your desired drop off goal. The robot has to travel to the
# desired pickup zone, display a message that it reached its destination,
# wait 5 seconds, travel to the desired drop off zone, and display a 
# message that it reached the drop off zone.
#
################################################################

. ./colors.sh	# enable terminal colors

function launch() {
	if [[ "$2" != "" ]]
	then
		sleep $2
	fi
	echo -e ${GREEN}"$1"${NC}
	if [[ "$2" == "0" ]]
	then
		xterm -e "$1"
	else
		xterm -e "$1" &
	fi
}

echo -e ${BLUE}"Starting Add Markers Project"${NC}

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
launch "roslaunch ${project} amcl.launch " 5

if [[ "${teleop}" == "keyboard" ]]
then
	launch "roslaunch turtlebot_teleop keyboard_teleop.launch" 5
else
	launch "roslaunch turtlebot_interactive_markers  interactive_markers.launch" 5
fi

launch "rosrun add_markers add_markers"
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

