#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
// #include "zones.h" // Definitions of constants associated with pickup and drop off zones

// Ref.: http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals

/* Define a client for to send goal requests to the move_base server 
   a through SimpleActionClient
*/

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){

  int status = 1; // 

  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait 5 sec for (move_base) action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting 5 sec for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  // goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.frame_id = "map"; // indicating which is the fixed frame
  goal.target_pose.header.stamp = ros::Time::now();

  // First goal

  // Define a position and orientation for the robot to reach
  // we'll send a goal to the robot to move 1 meter forward
  // pose data type? Point or Quaternion ? Why use 'w' = 4th par. of quaternion! or is "theta" ?
  // goal.target_pose.pose.position.x = -3.0; // robot moves back 2 meters
  // goal.target_pose.pose.position.z = -3.14159; // rotates robot 180 degrees counterclockwise (robot ends pointed "down")
  // goal.target_pose.pose.position.y = -2.0;
  // goal.target_pose.pose.position.roll = 0.0;
  // goal.target_pose.pose.position.pitch = 0.0;
  // goal.target_pose.pose.position.yaw = -1.5708; // down 90 degrees
  goal.target_pose.pose.position.x = 10.0; // from "zones.h"
  goal.target_pose.pose.position.y = 10.0; // from "zones.h"
  goal.target_pose.pose.orientation.w = 1.0; // w=0 means quaternio; w=1 means Euler
  // 1,0 rad = 57.2958^o ? 
  // 90^o = 1.5708 rad
  // 135^o = 2.35619 rad
  // https://www.google.com/search?channel=fs&client=ubuntu&q=coverter+radianos+para+graus 

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pickup zone data to robot");
  ac.sendGoal(goal);
  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    status = 0;
    ROS_INFO("Ok. Pickup zone reached");

    ROS_INFO("Waiting 5 seconds (robot collecting object) ...");
    ros::Duration(5.0).sleep();

    // Second goal
    // Define a position and orientation for the robot to reach
    // we'll send a goal to the robot to move 1 meter forward
    // goal.target_pose.pose.position.x = 3.0;
    // goal.target_pose.pose.position.y = -3.0; // robot moves left 3 meters
    // goal.target_pose.pose.position.z = 0.0; // ends at zero degree angle
    // goal.target_pose.pose.position.roll = 0.0;
    // goal.target_pose.pose.position.pitch = 0.0;
    // goal.target_pose.pose.position.yaw = 0.0; // down 90 degrees
    goal.target_pose.pose.position.x = 4.0; // from "zones.h"
    goal.target_pose.pose.position.y = 4.0; // from "zones.h"
    goal.target_pose.pose.orientation.w = 1.0; // w=0 means quaternio; w=1 means Euler


    // Send the goal position and orientation for the robot to reach
    ROS_INFO("------");
    ROS_INFO("Sending Drop off zone data to robot");
    ac.sendGoal(goal);
    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      status = 0;
      ROS_INFO("Ok. Drop off zone reached");
    }
    else
    {
      status = 2;
      ROS_INFO("ERROR: fail to reach Drop off zone");
    }
  }
  else
  {
    status = 1;
    ROS_INFO("ERROR: fail to reach Pickup zone");
  }
  ROS_INFO("Shutting down pick_objects node");
  return status;
}