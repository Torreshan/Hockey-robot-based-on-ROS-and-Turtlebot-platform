#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtle_vis/myClass/TurtleClass.h>
#include "collision_prevention.h"

/*
void drive() {
    ros::init(argc, argv, "robot_driver");

    turtleSpace::TurtleClass turtleF;
    collision_prevention::collision_prevention col_prevention;

    ros::NodeHandle nh;
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);
    ros::Subscriber odom_sub = nh.subscribe("/mobile_base_controller/odom",100,
                                            &turtleSpace::TurtleClass::getTiagoPose,
                                            &turtleF);
    ros::Subscriber laser_sub = nh.subscribe("/scan",
                                             1,
                                             &col_prevention::scanCallback,
                                             &col_prevention);

    Vector3d turtleTIAGOPose;
    Vector3d turtleTIAGOPose_local;
    turtleTIAGOPose<<1,0,0;    // WHY-----------------------------------------------------

    turtleTIAGOPose_local=turtleTIAGOPose;
    turtleF.turtlePose_g=turtleTIAGOPose;  // WHY-----------------------------------------------------

    ros::Time ti, tf;
    ti=ros::Time::now();

    Matrix3d Kp;

    double p_g=0.0;

    ros::param::get("/turtle_gains/p_gain",p_g);
    ROS_INFO_STREAM("p_g= "<<p_g);

    Kp<<p_g,0  ,0,
            0  ,p_g,0,
            0  ,0  ,p_g;

    ROS_INFO_STREAM("Kp= \n"<<Kp);

    Vector3d turtlePose,turtlePose_old,turtleVel;
    Vector3d error;
    double dt;

    //INITIALIZE THE TURTLE POSE
    turtlePose<<1,0,0;
    turtlePose_old=turtlePose;
    turtleVel<<0,0,0;

    //DESIRED POSE
    Vector3d turtlePose_desired_local;
    turtleF.turtlePose_desired_g=turtlePose;
    turtlePose_desired_local=turtlePose;

    geometry_msgs::Twist msg2;

    Matrix2d K;
    K<<0.5,0,
       0, 0.5;

    Matrix2d T_g;
    T_g<<0.1,0.1,
            0.1,0.1;

    float d = 0.3;
    Vector2d turtleTIAGOPose_2d,turtleTIAGOPose_2d_Now;
    Vector2d turtleTIAGOPose_desired_local_2d;

    turtleTIAGOPose_2d << 1,1;                   // Why 1 ?-----------------------------------------
    turtleTIAGOPose_desired_local_2d << 1,1;


    Vector2d error_2d;
    Vector2d turtleVel_2d;
    turtleVel_2d<<0,0;

    ros::Rate rate(10);
    ROS_INFO("Start moving");

    while(ros::ok())
    {
        tf=ros::Time::now();
        dt=tf.toSec()-ti.toSec();
        turtleTIAGOPose_local=turtleF.getLocalPose();

        //TEST
        std::cout<<"X: \t"<<turtleTIAGOPose_local[0]<<"Y: \t"<<turtleTIAGOPose_local[1]<<"Z: \t"<<turtleTIAGOPose_local[2]<<std::endl;
        turtleTIAGOPose_2d<<turtleTIAGOPose_local[0],turtleTIAGOPose_local[1];

        turtlePose_desired_local=turtleF.getLocalDesiredPose();

        turtleTIAGOPose_desired_local_2d<< turtlePose_desired_local[0], turtlePose_desired_local[1];

        error_2d=turtleTIAGOPose_desired_local_2d-turtleTIAGOPose_2d;

        turtleVel_2d=K*error_2d;

        T_g<< 1*cos(turtleTIAGOPose_local[2]),1*sin(turtleTIAGOPose_local[2]),
                -sin(turtleTIAGOPose_local[2])/d, cos(turtleTIAGOPose_local[2])/d;

        turtleTIAGOPose_2d_Now=T_g*turtleVel_2d;

        turtlePose= turtlePose + turtleVel*dt;  //USE SIMPLE INTEGRATION

        msg2.linear.x=turtleTIAGOPose_2d_Now(0);
        msg2.angular.z=turtleTIAGOPose_2d_Now(1);
        vel_pub.publish(msg2);

        turtlePose_old=turtlePose;
        ti=tf;

        ros::spin();
    }
}
*/

int main(int argc, char	**argv)	{
    ros::init(argc, argv, "robot_driver");
    //col_prevention.setTargetOdom(7.0, -7.0, 3.1415 / 2);

    ros::NodeHandle nh;

    turtleSpace::TurtleClass turtleF;
    collision_prevention col_prevention(turtleF);

    tf::TransformListener listener;
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);
    ros::Subscriber laser_sub = nh.subscribe("/lidarscan",
                                             1,
                                             &collision_prevention::scanCallback,
                                             &col_prevention);

    ros::ServiceServer service = nh.advertiseService("TurtlePose",
                                                       &turtleSpace::TurtleClass::getDPose,
                                                       &turtleF);


    ros::Subscriber sub = nh.subscribe("/mobile_base_controller/odom",100,
                                          &turtleSpace::TurtleClass::getTiagoPose,
                                          &turtleF);


    /*ros::Subscriber odom_sub = nh.subscribe("odom",
                                            1,
                                            &collision_prevention::odomCallback,
                                            &col_prevention);*/

    ros::Rate rate(10);
    ROS_INFO("Start moving");

    while (ros::ok()) {
        /*
        tf::StampedTransform transform;
        try {
            listener.lookupTransform("base_link",
                                     "target",
                                     ros::Time(0),
                                     transform);
        } catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
*/
        vel_pub.publish(col_prevention.getVelMsg());
        ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
        rate.sleep();

    }

    return 0;
}
