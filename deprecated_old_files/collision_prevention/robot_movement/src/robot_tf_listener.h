#ifndef ROBOT_TF_LISTENER_H
#define ROBOT_TF_LISTENER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>

class robot_tf_listener
{
public:
    robot_tf_listener(std::string self_name, std::string target_name);
    tf::StampedTransform listener_step();

private:
    std::string target_name;
    std::string self_name;
    tf::TransformListener listener;

};

#endif // ROBOT_TF_LISTENER_H
