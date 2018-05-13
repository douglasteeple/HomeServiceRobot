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
#include "nav_msgs/Odometry.h"

#ifndef PI
#define PI 3.14159265359
#endif
#ifndef HALFPI
#define HALFPI 1.57079632679
#endif
#ifndef QUARTPI
#define QUARTPI 0.785398163397
#endif

/*
 * Navigate to multiple goals and carry a marker from the pickup goal to
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
typedef boost::shared_ptr< ::move_base_msgs::MoveBaseFeedback const > MoveBaseFeedbackConstPtr;
typedef boost::shared_ptr< ::move_base_msgs::MoveBaseResult const > MoveBaseResultConstPtr;

visualization_msgs::Marker marker;
ros::Publisher marker_pub;

visualization_msgs::Marker createMarker(ros::Publisher *marker_pub) {
	ros::NodeHandle nodehandle;
	*marker_pub = nodehandle.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.
	marker.header.frame_id = "map";//"base_footprint";
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.
	marker.ns = "marker";
	marker.id = 0;

	// Set the marker type.
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker.mesh_resource = "package://home_service_project/meshes/CoffeeCup.dae";

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
	marker.scale.x = 0.125;
	marker.scale.y = 0.125;
	marker.scale.z = 0.125;

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

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
#ifdef TRACE
	ROS_INFO("Seq: [%d]", msg->header.seq);
	ROS_INFO("Position-> x: [%g], y: [%g], z: [%g]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
	ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
#endif
	//ROS_INFO("Moving marker to %g %g", msg->pose.pose.position.x, msg->pose.pose.position.y);
	marker.pose = msg->pose.pose;
}

void doneCb(const actionlib::SimpleClientGoalState& state, const MoveBaseResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
	ros::spinOnce();
}

// Called every time feedback is received for the goal
void feedbackCb(const MoveBaseFeedbackConstPtr& feedback)
{
	marker_pub.publish(marker);
	marker.pose.position = feedback->base_position.pose.position;
	marker.pose.position.z += 0.5;
	ros::spinOnce();
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
		ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

		// Wait 30 seconds for the results
		bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));

		// Check if the robot reached its goal
		if ((ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)) {
			ROS_INFO("Reached the %sgoal", (count==0?"final ":""));
			marker_pub.publish(marker);
			const int wait_time = 5;
			ROS_INFO("Waiting for %d seconds", wait_time);
			int msecs = wait_time*1000;
			while (msecs--) {
			  // defer to other callbacks
			  ros::spinOnce();
			  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.001));
			}
		} else {
			ROS_WARN("Failed move to the goal");
		}
	}
}

int main(int argc, char** argv){

	// Initialize the simple_navigation_goals node
	ros::init(argc, argv, "home_service");

	// Create a marker
	marker = createMarker(&marker_pub);

#ifdef USE_ODOM
	// Create odometer subscriber
	ros::NodeHandle nodehandle;
	ros::Subscriber sub = nodehandle.subscribe("odom", 1000, odomCallback);
#endif
	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	// Wait 5 sec for move_base action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	// wait so the screens can get organized
	ros::Duration(15.0).sleep(); // sleep for 15 seconds

	// send a vector of goals
	std::vector < geometry_msgs::Pose > poses;
	//poses.push_back(newPose(0.0, 1.0));
	poses.push_back(newPose(6.5, 0.0));
	//poses.push_back(newPose(0.0, 0.0));
	doSendGoalsAndWait(ac, marker, marker_pub, poses);

	// spin so the messages remain on the terminal screen...
	ros::spin();

	return 0;
}


