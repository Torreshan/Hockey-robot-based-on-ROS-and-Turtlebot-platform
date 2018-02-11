#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include<std_srvs/Empty.h>
#include<geometry_msgs/Twist.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
//  ros::NodeHandle n;
//  ros::ServiceClient client1 = n.serviceClient<std_srvs::Empty>("/global_localization");
//  std_srvs::Empty   srv1;
//  ros::Rate loop_rate(10);
//  ros::Duration(3).sleep();
//  if(client1.call(srv1)){
//    ROS_INFO("global localiztion");
//  }
//  else{
//      ROS_ERROR("Failed");

//  }
//  ros::Duration(3).sleep();
//  ros::Publisher pub=n.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel",100);
//  geometry_msgs::Twist msg;
//  for(int i = 0;i<250;i++){
//    msg.angular.z = 0.3;
//    pub.publish(msg);
//    ros::spinOnce();
//    loop_rate.sleep();
//}

//  ros::ServiceClient client2 = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
//  std_srvs::Empty srv2;
//  if(client2.call(srv2)){
//    ROS_INFO("clear");
//  }
//  else{
//      ROS_ERROR("Failed");

//  }

//  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

//  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 1;
  goal.target_pose.pose.position.y = 0;
  
  goal.target_pose.pose.orientation.z = 0;
  goal.target_pose.pose.orientation.w = 1;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved A");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

//  goal.target_pose.pose.position.x = -4.01;
//  goal.target_pose.pose.position.y = -6.39;
//  goal.target_pose.pose.orientation.z = 0.91;
//  goal.target_pose.pose.orientation.w = 0.43;
//  ROS_INFO("Sending goal");
//  ac.sendGoal(goal);

//  ac.waitForResult();

//  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//    ROS_INFO("Hooray, the base moved b");
//  else
//    ROS_INFO("The base failed to move forward 1 meter for some reason");


//    goal.target_pose.pose.position.x = -0.88;
//    goal.target_pose.pose.position.y = -12;
//    goal.target_pose.pose.orientation.z = 0.91;
//    goal.target_pose.pose.orientation.w = 0.43;
//    ROS_INFO("Sending goal");
//    ac.sendGoal(goal);

//    ac.waitForResult();

//    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//      ROS_INFO("Hooray, the base moved C");
//    else
//      ROS_INFO("The base failed to move forward 1 meter for some reason");

//    goal.target_pose.pose.position.x = -2.52;
//    goal.target_pose.pose.position.y = -2.24;
//    goal.target_pose.pose.orientation.z = 0.91;
//    goal.target_pose.pose.orientation.w = 0.43;
//    ROS_INFO("Sending goal");
//    ac.sendGoal(goal);

//    ac.waitForResult();

//    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//      ROS_INFO("Hooray, the base moved C");
//    else
//      ROS_INFO("The base failed to move forward 1 meter for some reason");

    return 0;
  }



