/*
 *      UDACITY ROBOTICS NANODEGREE PROGRAM TERM 2
 *                  Douglas Teeple
 *                    May 2018
 *
*/
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
/*
 * Navigate to two goals.
 * The first goal is the desired pickup goal and the second goal
 * is the desired drop off goal. The robot has to travel to the
 * desired pickup zone, display a message that it reached its destination,
 * wait 5 seconds, travel to the desired drop off zone, and display a 
 * message that it reached the drop off zone.
 * 
 */
// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){

  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.y = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending the first goal x=%g y=%g w=%g", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.orientation.w);
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, the base moved 1 meter forward");
    ROS_INFO("Waiting for 5 seconds");
    ros::Duration(5.0).sleep();
    goal.target_pose.pose.position.x = -1.0;
    goal.target_pose.pose.position.y = -2.0;
    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.header.stamp = ros::Time::now();
    ROS_INFO("Sending the second goal x=%g y=%g w=%g", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.orientation.w);
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, the base moved to the second goal");
    } else {
      ROS_WARN("The base failed to move forward 1 meter for some reason");
    }
  } else {
    ROS_WARN("The base failed to move forward 1 meter for some reason");
  }
  ros::spin();
  return 0;
}


