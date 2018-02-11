#ifndef COLLISION_PREVENTION_H
#define COLLISION_PREVENTION_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <turtle_vis/myClass/TurtleClass.h>


enum Decision {
    F,
    L,
    R,
    AGGRESIVE_L,
    AGGRESIVE_R,
    STOP,
    IDLE
};

struct Reservoir{
    float max_range;
    float avg_range;
    int max_range_index;
};

struct Range{
    float range;
    int index;
};

struct Odom{
    float x;
    float y;
    float yaw;
};


class collision_prevention
{
public:
    collision_prevention(const turtleSpace::TurtleClass &turtleF);

    turtleSpace::TurtleClass turtleF;

    const static bool ROS_INFO_ON = true;
    const static double MIN_SCAN_ANGLE_RAD = -40.0/180*M_PI;
    const static double MAX_SCAN_ANGLE_RAD = 40.0/180*M_PI;
    const static double MIN_FRONT_ANGLE_RAD = -12.0/180*M_PI;
    const static double MAX_FRONT_ANGLE_RAD = 12.0/180*M_PI;
    const static float MIN_PROXIMITY_RANGE_M = 0.5;
    const static float SIDE_AREA_ANGLE_RAD = 10.0/180*M_PI;
    const static float ANGLE_MIN = -3.12413907051;
    const static float ANGLE_INCREMENT = 0.0174532923847;

    geometry_msgs::Twist getVelMsg();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
    void setTargetOdom(const float &x,
                       const float &y,
                       const float &yaw);

private:
    //turtleSpace::TurtleClass turtleF;
    ros::ServiceServer service;
    ros::Subscriber sub;

    Odom target_odom;
    Odom base_odom;

    int right_index;
    int left_index;
    int right_front_index;
    int left_front_index;
    float mid_index;
    int lL_index;
    int rL_index;
    int lR_index;
    int rR_index;

    bool makingDecision;
    double forward_speed_mps;
    float correct_theta_rad;
    float ang_z;
    float dodge_ang_z;
    Decision decision;
    Decision previous_decision;

    sensor_msgs::LaserScan::ConstPtr scan;

    void makeDecision(const Reservoir &left_rsv,
                      const Reservoir &right_rsv,
                      const Range &closest_front_range,
                      const Range &closest_range,
                      const int &mid_index);
    Range getClosestRange(const sensor_msgs::LaserScan::ConstPtr& scan,
                          const int &right_index,
                          const int &left_index);
    Reservoir getReservoir(const sensor_msgs::LaserScan::ConstPtr& scan,
                            const int &right_index,
                            const int &left_index);
    float quaternionToYaw(const float &x,
                          const float &y,
                          const float &z,
                          const float &w);
    void moveForward();
};

#endif // COLLISION_PREVENTION_H
