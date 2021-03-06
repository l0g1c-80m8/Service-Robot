#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
    // Initialize the simple_navigation_goals node
    ros::init(argc, argv, "pick_objects");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("INFO: Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    // set up the frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = -3.25;
    goal.target_pose.pose.position.y = -4.50;
    goal.target_pose.pose.orientation.w = 1.0;

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("INFO: Sending pickup goal");
    ac.sendGoal(goal);
    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("SUCCESS: Robot arrived at pickup point");

        ROS_INFO("INFO: Robot is busy collecting object");
        ros::Duration(5.0).sleep();

        // Second goal
        goal.target_pose.pose.position.x = -4.50;
        goal.target_pose.pose.position.y = 7.25;
        goal.target_pose.pose.orientation.w = 1.0;


        // Send the goal position and orientation for the robot to reach
        ROS_INFO("INFO: Sending drop-off goal");
        ac.sendGoal(goal);
        // Wait an infinite time for the results
        ac.waitForResult();

        // Check if the robot reached its goal
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("SUCCESS: Robot arrived at drop-off point");

            // Make robot move away slightly after dropping off the object
            goal.target_pose.pose.position.x = -3.00;
            goal.target_pose.pose.position.y = 6.0;
            goal.target_pose.pose.orientation.w = 1.0;
            // Send the goal position and orientation for the robot to reach
            ROS_INFO("INFO: Moving robot away from drop-off point");
            ac.sendGoal(goal);
            // Wait an infinite time for the results
            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("SUCCESS: Robot successfully completed its mission");
            else
                ROS_INFO("ERROR: Robot failed its mission");

        }
        else
        {
            ROS_INFO("ERROR: Robot failed to reach drop-off point");
        }
    }
    else
    {
        ROS_INFO("ERROR: Robot failed to reach pickup point");
    }
    ros::Duration(10.0).sleep();
    ROS_INFO("INFO: Shutting down pick_objects node");
    return 0;
}
