#ifndef LANDMARKS_H
#define LANDMARKS_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// test eigen
//#include <eigen3/Eigen/Core>
//#include "laser_geometry.h"

class Stopper {
  public:
    // Tunable parameters
    const static double FORWARD_SPEED_MPS = 0.5;
    const static double MIN_SCAN_ANGLE_RAD = -30.0/180*M_PI;
    const static double MIN_SCAN_ANGLE_RAD_LEFT = -40.0/180*M_PI;
    const static double MAX_SCAN_ANGLE_RAD = +30.0/180*M_PI;
    const static double MAX_SCAN_ANGLE_RAD_RIGHT = +40.0/180*M_PI;
    const static float MIN_PROXIMITY_RANGE_M = 0.5; // Should be smaller than sensor_msgs::LaserScan::range_max
    const static float MIN_PROXIMITY_RANGE_SAFE = 1.5;
    const static double TURN_SPEED_RPS = 0.6;

    Stopper();
    void startMoving();

  private:
    ros::NodeHandle node;
    ros::Publisher commandPub; // Publisher to the robot's velocity command topic
    ros::Subscriber laserSub; // Subscriber to the robot's laser scan topic
    ros::Subscriber poseSub;
    bool keepMoving; // Indicates whether the robot should continue moving
    bool turnRightFlag;
    bool turnLeftFlag;
    bool turn;
    bool block;

    // void moveForward();
    // void turnLeft();
    // void turnRight();
    void motion(bool forward = false, bool left = false, bool right = false);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);
};

#endif // STOPPER_H
