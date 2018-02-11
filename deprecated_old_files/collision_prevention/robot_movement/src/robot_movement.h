#ifndef ROBOT_MOVEMENT_H
#define ROBOT_MOVEMENT_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include "robot_tf_listener.h"

class robot_movement
{
public:
    robot_movement();
    const static double FORWARD_SPEED_MPS = 0.15;
    const static double MIN_SCAN_ANGLE_RAD = -50.0/180*M_PI;
    const static double MAX_SCAN_ANGLE_RAD = +50.0/180*M_PI;
    const static float MIN_PROXIMITY_RANGE_M = 0.15; // Should be smaller than sensor_msgs::LaserScan::range_max
    const static float SIDE_AREA_ANGLE_RAD = 5.0/180*M_PI;

    const static std::string LASER_TOPIC_NAME = "/scan";
    const static std::string TARGET_TOPIC_NAME = ""; // TODO
    const static std::string VEL_COMMAND_TOPIC_NAME = "/cmd_vel_mux/input/teleop";

private:
    ros::NodeHandle node;
    ros::Publisher commandPub; // Publisher to the robot's velocity command topic
    ros::Subscriber laserSub; // Subscriber to the robot's laser scan topic
    ros::Subscriber targetSub; // Subscriber to ball and goal position
    bool keepMoving; // Indicates whether the robot should continue moving
    float correctThetaRad;
    float angZ;
    float dodgeAngZ;

    struct reservoir{
        float maxRange;
        float avgRange;
        int maxRangeIndex;
    };

    void start_moving();
    void moveForward();
    void turn(float turnAngleRad);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void targetCallback(const geometry_msgs::Point::ConstPtr& target);

    reservoir getReservoir(const sensor_msgs::LaserScan::ConstPtr& scan,
                           int minIndex,
                           int maxIndex);
};

#endif // ROBOT_MOVEMENT_H
