#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "stopper.h"

Stopper::Stopper() {
  keepMoving = true;
  correctThetaRad = 0.0;
  angZ = 0.0;
  dodgeAngZ = 0.0;

  // Advertise a new publisher for the simulated robot's velocity command topic
  commandPub = node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);

  // Subscribe to the simulated robot's laser scan topic
  laserSub = node.subscribe("/scan", 1, &Stopper::scanCallback, this);
}

// Send a velocity command
void Stopper::moveForward() {
  geometry_msgs::Twist msg; // The default constructor will set all commands to 0
  msg.linear.x = FORWARD_SPEED_MPS;
  msg.angular.z = angZ;
  commandPub.publish(msg);
}

void Stopper::turn(float turnAngleRad) {
    geometry_msgs::Twist msg;
    msg.linear.x = 0.0;
    msg.angular.z = turnAngleRad;
    commandPub.publish(msg);
}

// Process the incoming laser scan message
void Stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // Find the closest range between the defined minimum and maximum angles
    int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
    float midIndex = (maxIndex + minIndex) / 2.0;

    // Not sure with the indexing though!
    int minLeftSideIndex = maxIndex + 1;
    int maxLeftSideIndex = minLeftSideIndex + floor(SIDE_AREA_ANGLE_RAD / scan->angle_increment);
    int maxRightSideIndex = minIndex - 1;
    int minRightSideIndex = maxRightSideIndex - floor(SIDE_AREA_ANGLE_RAD / scan->angle_increment);


  float closestRange = scan->ranges[minIndex];
  int closestIndex = minIndex;
  for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
      if (scan->ranges[currIndex] < closestRange) {
          closestRange = scan->ranges[currIndex];
          closestIndex = currIndex;
        }
  }

  // Evaluate left side reservoir
  reservoir leftRsv = getReservoir(scan, minLeftSideIndex, maxLeftSideIndex);

  // Evaluate right side reservoir
  reservoir rightRsv = getReservoir(scan, minRightSideIndex, maxRightSideIndex);


  //ROS_INFO_STREAM("Closest range: " << closestRange);

  // Check if there is more space on the left/right side than the front side. If yes, go to that direction.
  if (closestRange > leftRsv.avgRange && closestRange > rightRsv.avgRange)
  {
      ROS_INFO_STREAM("Strategy: forward");
      angZ = 0.0;
  }
  else if (leftRsv.avgRange < rightRsv.avgRange && closestRange > MIN_PROXIMITY_RANGE_M * 2)
  {
      ROS_INFO_STREAM("Strategy: right");
      correctThetaRad = abs(rightRsv.maxRangeIndex - closestIndex) * scan->angle_increment;
      correctThetaRad = fmin(2*M_PI - correctThetaRad, correctThetaRad);
      float tApproach = (closestRange / FORWARD_SPEED_MPS);
      angZ = - correctThetaRad / tApproach - 0.1;
  }
  else if (leftRsv.avgRange > rightRsv.avgRange && closestRange > MIN_PROXIMITY_RANGE_M * 2)
  {
      ROS_INFO_STREAM("Strategy: left");
      correctThetaRad = abs(leftRsv.maxRangeIndex - closestIndex) * scan->angle_increment;
      correctThetaRad = fmin(2*M_PI - correctThetaRad, correctThetaRad);
      float tApproach = (closestRange / FORWARD_SPEED_MPS);
      angZ = correctThetaRad / tApproach + 0.1;
  }
  else if ( closestRange < 2 * MIN_PROXIMITY_RANGE_M && closestRange > MIN_PROXIMITY_RANGE_M )
  {
      if (closestIndex > midIndex) // Obstacle on the left side, so turn right.
      {
          ROS_INFO_STREAM("Strategy: dodge right");
          correctThetaRad = abs(rightRsv.maxRangeIndex - closestIndex) * scan->angle_increment;
          correctThetaRad = fmin(2*M_PI - correctThetaRad, correctThetaRad);
          float tApproach = (closestRange / FORWARD_SPEED_MPS);
          if (tApproach < 0.5) {
              angZ = - M_PI / 2;
          } else {
              angZ = - correctThetaRad / tApproach - 0.1;
          }

      } else {
          ROS_INFO_STREAM("Strategy: dodge left");
          correctThetaRad = abs(leftRsv.maxRangeIndex - closestIndex) * scan->angle_increment;
          correctThetaRad = fmin(2*M_PI - correctThetaRad, correctThetaRad);
          float tApproach = (closestRange / FORWARD_SPEED_MPS);
          if (tApproach < 0.5) {
              angZ = - M_PI / 2;
          } else {
              angZ = correctThetaRad / tApproach + 0.1;
          }
    }
  }

  if (closestRange < MIN_PROXIMITY_RANGE_M)
  {
      ROS_INFO_STREAM("Stop!");
      if (keepMoving) {
          if (closestIndex > midIndex){
              dodgeAngZ = - M_PI / 2;
          } else {
              dodgeAngZ = M_PI / 2;
          }
      }
      keepMoving = false;
      turn(dodgeAngZ);
  }
  else
      keepMoving = true;

}


Stopper::reservoir Stopper::getReservoir(const sensor_msgs::LaserScan::ConstPtr& scan,
                                         int minIndex,
                                         int maxIndex)
{
    reservoir rsv;

    rsv.maxRange = scan->ranges[minIndex];
    rsv.maxRangeIndex = minIndex;
    rsv.avgRange = 0;
    int count = 0;
    for (int currIndex = minIndex + 1; currIndex < maxIndex; currIndex++) {
        rsv.avgRange += scan->ranges[currIndex];
        count++;
        if (scan->ranges[currIndex] > rsv.maxRange) {
            rsv.maxRange = scan->ranges[currIndex];
            rsv.maxRangeIndex = currIndex;
        }
    }
    rsv.avgRange /= count;
    return rsv;
}


void Stopper::startMoving()
{
  ros::Rate rate(10);
  ROS_INFO("Start moving");

  // Keep spinning loop until user presses Ctrl+C
  while (ros::ok()) {
      // Continue moving if scan changes (we or the obstacle gets moved)
      if (keepMoving) {
          moveForward();
      }

      ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
      rate.sleep();
    }
}
int main(int argc, char	**argv)	{
  // Initiate new ROS node named "stopper"
  ros::init(argc, argv,	"stopper");

  // Create new stopper object
  Stopper stopper;

  // Start the movement
  stopper.startMoving();

  return 0;
}

