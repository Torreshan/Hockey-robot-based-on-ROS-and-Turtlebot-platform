#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include "stopper.h"

Stopper::Stopper() 
{
  keepMoving = true;
  turn = false;
  turnLeftFlag = false;
  turnRightFlag = false;

  // Advertise a new publisher for the simulated robot's velocity command topic
  commandPub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  // Subscribe to the simulated robot's laser scan topic
  laserSub = node.subscribe("base_scan", 1, &Stopper::scanCallback, this);
  poseSub = node.subscribe("amcl_pose", 1, &Stopper::poseCallback, this);
}

void Stopper::motion(bool forward, bool left, bool right)
{
  geometry_msgs::Twist msg;
  if(forward)
    msg.linear.x = FORWARD_SPEED_MPS;
  if(left)
    msg.angular.z = TURN_SPEED_RPS;
  if(right)
    msg.angular.z = -TURN_SPEED_RPS;
  commandPub.publish(msg);
}

void Stopper::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
  double x = pose->pose.pose.position.x;
  double y = pose->pose.pose.position.y;

  // ROS_INFO("position: x: %f, y: %f",x, y);

}
// Process the incoming laser scan message
void Stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // Find the closest range between the defined minimum and maximum angles
  int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
  int minIndexLeft = ceil((MIN_SCAN_ANGLE_RAD_LEFT - scan->angle_min) / scan->angle_increment);
  int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
  int maxIndexRight = floor((MAX_SCAN_ANGLE_RAD_RIGHT - scan->angle_min) / scan->angle_increment);

  float closestRange = scan->ranges[minIndex];
  for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) 
  {
    if (scan->ranges[currIndex] < closestRange) 
    {
      closestRange = scan->ranges[currIndex];
    }
  }

  // chekck the left outside range
  float closestRangeLeft = scan->ranges[minIndex];
  for (int currIndex = minIndex -1; currIndex >= minIndexLeft; currIndex--) 
  {
    if (scan->ranges[currIndex] < closestRangeLeft) 
    {
      closestRangeLeft = scan->ranges[currIndex];
    }
  } 
  // check the right outside range
  float closestRangeRight = scan->ranges[maxIndex];
  for (int currIndex = maxIndex +1; currIndex <= maxIndexRight; currIndex++) 
  {
    if (scan->ranges[currIndex] < closestRangeRight) 
    {
      closestRangeRight = scan->ranges[currIndex];
    }
  }
  ROS_INFO_STREAM("Closest Range: " << closestRange << "Left" << closestRangeLeft << "Right" << closestRangeRight);


  if (closestRange < MIN_PROXIMITY_RANGE_SAFE) 
    turn = true;
  else
    turn = false;

  if (closestRange < MIN_PROXIMITY_RANGE_M) 
  {
    // ROS_INFO("Stop!");
    keepMoving = false;
    turn = true;
  }
  else
    keepMoving = true;

  if (turn == true && closestRangeLeft <= closestRangeRight)
  {
    // ROS_INFO("Turn Left!");
    turnLeftFlag = true;
    turnRightFlag = false;
  }
  else if (turn == true && closestRangeLeft > closestRangeRight)
  {
    // ROS_INFO("Turn Right!");
    turnLeftFlag = false;
    turnRightFlag = true;
  }
  ROS_INFO_STREAM(" keepMoving " << keepMoving << " turn " << turn << " left " << turnLeftFlag << " right " << turnRightFlag);
}

void Stopper::startMoving()
{
  ros::Rate rate(10);
  ROS_INFO("Start moving");

  // Keep spinning loop until user presses Ctrl+C
  while (ros::ok()) 
  {
    // Continue moving if scan changes (we or the obstacle gets moved)
    if (keepMoving && !turn)
      motion(true, false, false);
    if (keepMoving && turn && turnLeftFlag)
      motion(true, true, false);
    if (keepMoving && turn && turnRightFlag)
      motion(true, false, true); 
    if(!keepMoving && turnLeftFlag)
      motion(false, true, false);
    if(!keepMoving && turnRightFlag)
      motion(false, false, true);

    ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
    rate.sleep();
  }
}
