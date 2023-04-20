#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <actionlib/client/simple_action_client.h>

double pgoal[2] = {0, 0};
// double dgoal[2] = {0, 0};

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void marker_callback_pickup(const visualization_msgs::Marker::ConstPtr& msg)
{
  pgoal[0] = msg->pose.position.x;
  pgoal[1] = msg->pose.position.y;
  ROS_INFO("Update goal %f %f", pgoal[0], pgoal[1]);
}

// void marker_callback_dropoff(const visualization_msgs::Marker::ConstPtr& msg)
// {
//   dgoal[0] = msg->pose.position.x;
//   dgoal[1] = msg->pose.position.y;
//   ROS_INFO("Update goal %f %f", pgoal[0], pgoal[1]);
// }

void send_pickup_goal(MoveBaseClient & ac, double x, double y){
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = 1;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pick up goal to the robot ... %f, %f", pgoal[0], pgoal[1]);
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Wow the robot has reached the goal!");
  else
    ROS_INFO("The base failed to reach the goal for some reason.");
  
}

void send_dropoff_goal(MoveBaseClient & ac, double x, double y){

   
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = 1;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending drop off goal to the robot ... %f, %f", pgoal[0], pgoal[1]);
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Woww robot dropped the virtual object at the drop off location!");
  else
    ROS_INFO("The base failed to reach the goal for some reason.");


  
}

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  ros::NodeHandle n;
  ros::Subscriber marker_sub = n.subscribe("visualization_marker", 10, marker_callback_pickup);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  

  ros::spinOnce();
  
  send_pickup_goal(ac, pgoal[0], pgoal[1]);

  ros::Duration(5).sleep(); // simulate the picking up
 
  // ros::NodeHandle n1;
  // ros::Subscriber marker_sub1 = n1.subscribe("visualization_marker", 10, marker_callback_pickup);
  ros::spinOnce();
  send_dropoff_goal(ac, pgoal[0], pgoal[1]);

  

  ROS_INFO("Now reached to the drop off area");

  return 0;
}