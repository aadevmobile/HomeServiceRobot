#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "add_markers/setupMarker.h"

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");
  // marker service client
  ros::NodeHandle n;
  ros::ServiceClient setupMarkerClient = n.serviceClient<add_markers::setupMarker>("/add_marker/setupMarker");
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal pickUp;
  move_base_msgs::MoveBaseGoal dropOff;

  // set up the frame parameters
  pickUp.target_pose.header.frame_id = "odom";
  pickUp.target_pose.header.stamp = ros::Time::now();

  dropOff.target_pose.header.frame_id = "odom";
  dropOff.target_pose.header.stamp = ros::Time::now();
  
  // Define a position and orientation for the robot to reach
  pickUp.target_pose.pose.position.x = 2.8;
  pickUp.target_pose.pose.position.y = 5;
  pickUp.target_pose.pose.orientation.w = 1.0;

  dropOff.target_pose.pose.position.x = -2.07;  
  dropOff.target_pose.pose.position.y = -0.5;
  dropOff.target_pose.pose.orientation.w = 1.57;
  
  // add marker to pickUp location
  add_markers::setupMarker marker;
  marker.request.position_x = pickUp.target_pose.pose.position.x;
  marker.request.position_y = pickUp.target_pose.pose.position.y;
  marker.request.oppacity = 1.0;
  setupMarkerClient.call(marker);

  // Send the initial position and orientation for the robot to reach
  ROS_INFO("Sending pickUp coordinates");
  ac.sendGoal(pickUp);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {   
     ROS_INFO("Hooray, pick up move completed");
     
     // Wait 5 sec for move_base action server to come up
     while(!ac.waitForServer(ros::Duration(5.0))){
     ROS_INFO("Waiting for the move_base action server to come up");
     }

     // Hide Marker
     marker.request.oppacity = 0.0;
     setupMarkerClient.call(marker);

     // Send the initial position and orientation for the robot to reach
     ROS_INFO("Sending dropOff coordinates");
     ac.sendGoal(dropOff);

     // wait an infinite time for the results
     ac.waitForResult();

     // Check if the robot reached its goal
     if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
     ROS_INFO("Hooray, pick up and dropOff completed!");
        // show marker in dropOff position
        marker.request.position_x = dropOff.target_pose.pose.position.x;  
        marker.request.position_y = dropOff.target_pose.pose.position.y;
        marker.request.oppacity = 1.0;
        setupMarkerClient.call(marker);     

     }
     else{
     ROS_INFO("DropOff Failed :/");
	return 0;
     }
  }else
    ROS_INFO("PickUp Failed :/");

  return 0;
}

