/*********************************************************************
* Compiler:         gcc 4.6.3
*
* Company:          Institute for Cognitive Systems
*                   Technical University of Munich
*
* Author:           Emmanuel Dean (dean@tum.de)
*                   Karinne Ramirez (karinne.ramirez@tum.de)
*
* Compatibility:    Ubuntu 12.04 64bit (ros hydro)
*
* Software Version: V0.1
*
* Created:          01.06.2015
*
* Comment:          turtle connection and visualization (Sensor and Signals)
*
********************************************************************/


/*********************************************************************
* STD INCLUDES
********************************************************************/
#include <iostream>
#include <fstream>
#include <pthread.h>
#include <math.h>


/*********************************************************************
* ROS INCLUDES
********************************************************************/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
//#include <tf_conversions/tf_eigen.h>

/*********************************************************************
 * CUSTOM CLASS
 * ******************************************************************/
#include <turtle_vis/myClass/TurtleClass.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
using namespace Eigen;
bool gk = false;
bool callback(std_srvs::Empty::Request& e, std_srvs::Empty::Response& ee){
     bool gk = true;


     return true;
}


int main( int argc, char** argv )
{

    ros::init(argc, argv, "turtle_control",ros::init_options::AnonymousName);

    ROS_INFO_STREAM("**Publishing turtle control..");

    ros::NodeHandle n;
    ros::Rate r(60);


    //ADVERTISE THE SERVICE
    turtleSpace::TurtleClass turtleF;
    //turtleSpace::TurtleClass turtleff;
    ros::ServiceServer service=n.advertiseService("TurtlePose",
                                                  &turtleSpace::TurtleClass::getDPose,//#>>>>TODO: DEFINE THE CALLBACK FUNCTION,
                                                  &turtleF);
    ros::ServiceServer service1=n.advertiseService("gogogo",callback);

    //CALL SERVICE FROM TERMINAL//
    //    rosservice call /TurtlePose '{p: [0.5, 0.0, 3.0]}'
    //    rosservice call /TurtlePose "{p: {x: 1.5, y: 1.0, theta: 0.0}}"
    //DON'T FORGET TO SOURCE THE WORKSPACE BEFORE CALLING THE SERVICE
    //ros::Publisher pub = n.advertise<nav_msgs::Odometry>("tiago_control",100);
    //ADVERTIZE THE TOPIC
    ros::Publisher pub=n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",100);
    ros::Subscriber sub = n.subscribe("/odom", 100, &turtleSpace::TurtleClass::getTiagoPose, &turtleF);
/*#>>>>DEFINE THE MSG TYPE*/
    ros::Time ti, tf;
    ti=ros::Time::now();

    //Proportional Gain
    Matrix2d Kp;

   	

    //#>>>>SET GAINS

    double p_g=0.3;

    //#>LOAD p_gain FROM THE ROS PARAMETER SERVER
    
    //ros::param::get("p_gain",p_g);
    //ROS_INFO_STREAM("p_g= "<<p_g);


    //Proportional Gain

//    Kp<<p_g,0  ,0,
//            0  ,p_g,0,
//            0  ,0  ,p_g;
    Kp<<p_g,0,
            0,p_g;

    ROS_INFO_STREAM("Kp= \n"<<Kp);

    Vector3d turtlePose,turtlePose_old;
    Vector2d error,turtleVel;
    double dt;

    //INITIALIZE THE TURTLE POSE
    turtlePose = turtleF.getLocalPose();
    //turtlePose_old=turtlePose;
    turtleVel<<0,0;

    //DESIRED POSE
    Vector3d turtlePose_desired_local;
    double d = 0.3;
    ////#>>>>TODO: INITIALIZE THE DESIRED POSE VARIABLE OF THE CLASS TURTLE
    //turtleff.turtlePose_g=turtlePose;
    turtlePose_desired_local=turtleF.getLocalPose();

    //CREATE A DESIREDPOSE MSG VARIABLE
    geometry_msgs::Twist msg; //#>>>>DEFINE THE MSG TYPE
    Vector2d tiagopose, tiagodpose, T;
    double theta1, thetadesire,tt;
    Matrix2d angular;
    while(ros::ok())
    {
        if(0){
        tf=ros::Time::now();

        dt=tf.toSec()-ti.toSec();
        turtlePose = turtleF.getLocalPose();
        tiagopose[0] = turtlePose[0];
        tiagopose[1] = turtlePose[1];
        theta1 = turtlePose[2];
        ////#>>>> Get Desired Pose from the class variable
        turtlePose_desired_local=turtleF.getLocalDesiredPose();
//        tiagodpose[0] = turtlePose_desired_local[0] + d*cos(turtlePose_desired_local[2]);
//        tiagodpose[1] = turtlePose_desired_local[1] + d*sin(turtlePose_desired_local[2]);
        tiagodpose[0] = turtlePose_desired_local[0] ;
        tiagodpose[1] = turtlePose_desired_local[1] ;
        //std::cout<< tiagodpose[0] <<"kkkk"<< tiagodpose[1] <<std::endl;
        thetadesire = turtlePose_desired_local[2];
        //CONTROL
        ////#>>>>TODO:COMPUTE THE ERROR BETWEEN CURRENT POSE AND DESIRED
        error=tiagopose - tiagodpose;
        // COMPUTE THE INCREMENTS
        turtleVel=-Kp*error;
        tt = -p_g*(theta1 - thetadesire);
//        angular(0,0) = cos(theta1);
//        angular(0,1) = d*sin(theta1);
//        angular(1,0) = sin(theta1);
//        angular(1,1) = d*cos(theta1);
        angular(0,0) = d*cos(theta1);
        angular(0,1) = d*sin(theta1);
        angular(1,0) = -sin(theta1);
        angular(1,1) = cos(theta1);
        ////#>>>>TODO:COMPUTE THE NEW TURTLE POSE
        T = angular*turtleVel/d;//USE SIMPLE INTEGRATION;
        //Publish Data
        ////#>>>>TODO:SET THE MSG VARIABLE WITH THE NEW TURTLE POSE
        msg.linear.x = T[0];
        
        msg.angular.z = T[1];
    
        //(if((T[0]<0.000000001)&&(T[1]<0.000000001))
        //    continue;
        //pub.publish(msg);
        /*if(T[0]<0.000000001){          
            msg.linear.x = 0;
            // std::cout << 11<<std::endl;
      }
          else{
           msg.linear.x = T[0];
           std::cout << 1 <<std::endl;
       }*/
           


          // if(T[1]<0.1){
          //msg.angular.z = 0.001;
      //}
        //else{

        //msg.linear.z = T[1];
    //}
        pub.publish(msg);
        //SET THE HISTORY
        //if((T[1]==0)&& (T[0]==0))
           // gk = false;
        ti=tf;
    }
       //ROS::SPIN IS IMPORTANT TO UPDATE ALL THE SERVICES AND TOPICS
        ros::spin();

        r.sleep();
    
    }
    
    return 0;
}


