#include "robot_tf_listener.h"

robot_tf_listener::robot_tf_listener(std::string self_name, std::string target_name)
{
    this->target_name = target_name;
    this->self_name = self_name;
}

tf::StampedTransform robot_tf_listener::listener_step()
{
    tf::StampedTransform targetTransform;

    try {
        this->listener.lookupTransform(this->self_name, this->target_name, ros::Time(0), targetTransform);
    } catch (tf::TransformException e) {
        ROS_ERROR("%s", e.what());
        ros::Duration(1.0).sleep();
    }

    return targetTransform;
}

