#ifndef STOPPER_H
#define STOPPER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class Stopper {
  public:
    // Tunable parameters
    const static double FORWARD_SPEED_MPS = 0.15;
    const static double MIN_SCAN_ANGLE_RAD = -50.0/180*M_PI;
    const static double MAX_SCAN_ANGLE_RAD = +50.0/180*M_PI;
    const static float MIN_PROXIMITY_RANGE_M = 0.2; // Should be smaller than sensor_msgs::LaserScan::range_max
    const static float SIDE_AREA_ANGLE_RAD = 5.0/180*M_PI;
    //const static double SCHULTERBLICK_ANGLE_RAD = 120.0/180*M_PI;

    Stopper();
    void startMoving();

  private:
    ros::NodeHandle node;
    ros::Publisher commandPub; // Publisher to the robot's velocity command topic
    ros::Subscriber laserSub; // Subscriber to the robot's laser scan topic
    bool keepMoving; // Indicates whether the robot should continue moving
    float correctThetaRad;
    float angZ;
    float dodgeAngZ;

    struct reservoir{
        float maxRange;
        float avgRange;
        int maxRangeIndex;
    };

    void moveForward();
    void turn(float turnAngleRad);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

    reservoir getReservoir(const sensor_msgs::LaserScan::ConstPtr& scan,
                           int minIndex,
                           int maxIndex);

};

#endif // STOPPER_H
