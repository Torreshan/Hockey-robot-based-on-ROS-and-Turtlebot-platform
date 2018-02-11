#ifndef LANDMARKS_H
#define LANDMARKS_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <vector>
#include <geometry_msgs/Point.h>

//#include "Features/featureArray.h"
//#include "Features/featureArrayH.h"




class Landmarks {
  public:
//some constant
    const static double RANGE_THRESHOLD = 10.0;
    const static double R_POLE = 0.06;
    const static double THRESHOLD_SAME_OBJ_R = 0.1;
    const static double THRESHOLD_SAME_OBJ_A = 1.5*0.0174532923847+0.003;
    const static double THRESHOLD_MAX_R = 3;


    // constructor
    Landmarks();


  private:

    bool debug = false;

    int count = 0;

    // position
    struct position{
        double x;
        double y;
    };

    std::vector<position> positionList;


//    localization::featureArrayH feature_array;
    geometry_msgs::Point landmarkPos;

    // LaserScan msg
    sensor_msgs::LaserScan scanFilter;


    ros::NodeHandle node_;
//    laser_geometry::LaserProjection projector_;
//    tf::TransformListener tfListener_;
    ros::Subscriber laserSub_; // Subscriber to the robot's laser scan topic
//    ros::Publisher featurePub_;
    ros::Publisher laserPub_;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

};

#endif
