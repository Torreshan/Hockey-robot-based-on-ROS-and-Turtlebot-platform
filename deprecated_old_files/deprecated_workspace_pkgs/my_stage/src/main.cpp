#include <ros/ros.h>
#include "landmarks.h"
//#include <geometry_msgs/PointStamped.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "landmarks");

    Landmarks landmarks;

    ros::spin();

    return 0;
}
//geometry_msgs::PointStampedConstPtr


