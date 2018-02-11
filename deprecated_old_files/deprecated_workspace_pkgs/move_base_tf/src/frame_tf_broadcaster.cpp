#include <ros/ros.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
void Callback(const geometry_msgs::PointStampedConstPtr& pp){
    tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(pp->point.x, pp->point.y, pp->point.z) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "goal_pose"));
    std::cout<< 1 <<std::endl;
  
}
int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;

  

  ros::Rate rate(10.0);
  ros::Subscriber sub = node.subscribe("/3D_point", 100, &Callback);
  std::cout<< 1 <<std::endl;
  ros::spin();
  rate.sleep();
  return 0;
}
 