/*
 *      UDACITY ROBOTICS NANODEGREE PROGRAM TERM 2
 *                  Douglas Teeple
 *                    May 2018
 *
*/
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

/*
 * Navigate to two goals and move a marker from the pickup goal to
 * the dropoff goal.
 * The first goal is the desired pickup goal and the second goal
 * should be your desired drop off goal. The robot has to travel to the
 * desired pickup zone, display a message that it reached its destination,
 * wait 5 seconds, travel to the desired drop off zone, and display a 
 * message that it reached the drop off zone.
 * 
 */

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

visualization_msgs::Marker createMarker(ros::Publisher *marker_pub) {
	ros::NodeHandle nodehandle;
	*marker_pub = nodehandle.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.
	marker.ns = "marker";
	marker.id = 0;

	// Set the marker type. TEXT_VIEW_FACING
	marker.type = visualization_msgs::Marker::CUBE;

	// Set the marker action.
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker
	marker.scale.x = 0.25;
	marker.scale.y = 0.25;
	marker.scale.z = 0.25;

	// Set the color
	// Dodger blue: 0.118, 0.565, 1.000
	// Gold: 1.000, 0.843, 0.000
	marker.color.r = 1.0;
	marker.color.g = 0.843;
	marker.color.b = 0.0;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();
	return marker;
}

const geometry_msgs::Pose newPose(float px, float py, float pz=0, float qx=0, float qy=0, float qz=0, float qw=1) {
	geometry_msgs::Pose pose;
	pose.position.x = px;
	pose.position.y = py;
	pose.position.z = pz;
	pose.orientation.x = qx;
	pose.orientation.y = qy;
	pose.orientation.z = qz;
	pose.orientation.w = qw;
	return pose;
}

bool doSendGoalsAndWait(MoveBaseClient &ac, visualization_msgs::Marker &marker, ros::Publisher &marker_pub, std::vector < geometry_msgs::Pose > const &posevector) {
	move_base_msgs::MoveBaseGoal goal;

	while (marker_pub.getNumSubscribers() < 1) {
		if (!ros::ok()) {
			return 0;
		}
		ROS_WARN_ONCE("Please create a subscriber to the marker");
		sleep(1);
	}

	// set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	// Define a position and orientation for the robot to reach
	size_t count = posevector.size();
	for (const auto &pose : posevector) {
		marker.pose = goal.target_pose.pose = pose;
		count--;

		// Send the goal position and orientation for the robot to reach
		ROS_INFO("Sending the  goal x=%g y=%g w=%g", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.orientation.w);
		ac.sendGoal(goal);

		// Wait an infinite time for the results
		ac.waitForResult();

		// Check if the robot reached its goal
		if ((ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)) {
			ROS_INFO("Reached the %sgoal", (count==0?"final ":""));
			if (count == 0) {
				marker.action = visualization_msgs::Marker::ADD;
				marker_pub.publish(marker);
				break;
			}
			const float wait_time = 5.0;
			ROS_INFO("Waiting for %g seconds", wait_time);
			ros::Duration(wait_time).sleep();
			marker.action = visualization_msgs::Marker::DELETE;
			marker_pub.publish(marker);
		} else {
			ROS_WARN("The base failed to move to the goal");
			break;
		}
	}
}

int main(int argc, char** argv){

	// Initialize the simple_navigation_goals node
	ros::init(argc, argv, "add_markers");

	// Create a marker
	ros::Publisher marker_pub;
	visualization_msgs::Marker marker = createMarker(&marker_pub);

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	// Wait 5 sec for move_base action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	// send a vector of goals
	std::vector < geometry_msgs::Pose > poses;
	//poses.push_back(newPose(0.0, 1.0));
	poses.push_back(newPose(6.5, 0.0));
	//poses.push_back(newPose(0.0, 0.0));
	doSendGoalsAndWait(ac, marker, marker_pub, poses);

	// spin so we can see the messages...
	ros::spin();

	return 0;
}


