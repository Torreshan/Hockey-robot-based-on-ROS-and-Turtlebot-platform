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


/*********************************************************************
* ROS INCLUDES
********************************************************************/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
//#include <tf_conversions/tf_eigen.h>

/*********************************************************************
* EIGEN INCLUDES
********************************************************************/
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Core>

/*********************************************************************
 * SEVICES AND MESSAGES
 * ******************************************************************/
//SET HEADERS FOR THE SERVICE AND THE MESSAGES OF THE TURTLE_VIS PACKAGE
#include <turtle_vis/myClass/TurtleClass.h>
#include <turtle_vis/DesiredPose.h>
#include <turtle_vis/send_desired_pose.h>
#include <geometry_msgs/PointStamped.h>
using namespace Eigen;


geometry_msgs::PointStamped pp;

void backll(const geometry_msgs::PointStampedConstPtr& p){
      
     
    pp.point.x = p->point.x;
    pp.point.z = p->point.z;
    pp.point.y = p->point.y;


}
int main(int argc, char** argv)
{

    ros::init(argc, argv, "turtle_visualization",ros::init_options::AnonymousName);

    ROS_INFO_STREAM("**Client turtle desired position");

    ros::NodeHandle n;
    ros::Rate r(60);

    //INITIALIZE THE CLIENT
    ros::ServiceClient client=n.serviceClient<turtle_vis::send_desired_pose/*//#>>>>TODO: DEFINE THE SERVICE TYPE*/>("TurtlePose");

    ros::Subscriber sub = n.subscribe("/3D_point", 100, /*&geometry_msgs::PointStamped,*/&backll );

    ////#>>>>TODO: DEFINE A MSG VARIABLE FOR THE SERVICE MESSAGE
    turtle_vis::send_desired_pose srv;

    // turtle_vis::send_desired_pose msg;                // Publisher-> Subscribe msg    Client->Service Srv

    std::string myString;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion qtf;

    while(ros::ok())
    {
    if((pp.point.x != 0)|(pp.point.y != 0))
        std::vector<double> vals;

        ROS_INFO_STREAM("Give me the desired position of the turtle: x,y,theta");
        //std::cin>>myString;
        std::cout << pp.point.x<<std::endl;
        srv.request.a = pp.point.x;
        srv.request.b = pp.point.y;
        srv.request.c = 0; // theta


        //std::cout<<"x= "<<srv.request.a<<"\t y= "<< srv.request.b << "\t theta= "<<srv.request.c << std::endl;


        ////#>>>>TODO:GET THE VALUES FROM THE TERMINAL AND SAVE THEM IN A LOCAL VARIABLE. YOU WILL GET X,Y AND THETA
        //msg.request.a = std::stoi(myString);
        //msg.request.b =
        //msg.request.c =
//        float num3[3];

//        for(int i=0; i<3; i++)
//        {
//            std::cin>>num3[i];
//            vals.push_back(num3[i]);
//        }
        ////#>>>>TODO:CREATE THE MESSAGE WITH THE LOCAL VARIABLE

//        srv.request.a=num3[0];
//        srv.request.b=num3[1];
//        srv.request.c=num3[2];

        ////#>>>>TODO:COMPUTE THE POSITION AND ORIENTATION OF THE TF FOR THE DESIRED POSITION
        //qtf.setRPY(0,0,msg.request.c); ////#>>>>TODO:USE THETA VARIABLE
        


        if(client.call(srv))  //#>>>>TODO:CALL THE CLIENT WITH msg
        {
            //#>>>>TODO:PLOT THE MESSAGE
            //ROS_INFO_STREAM("ROS has been Sent!!!!");
            //std::cout<<"x= "<<srv.request.a<<"\t y= "<< srv.request.b << "\t theta= "<<srv.request.c << std::endl;

        }
        else
        {
            ROS_ERROR_STREAM("Failed to call the service 'TurtlePose'");
            return 1;
        }
     ros::spinOnce();
    }


    ros::spinOnce();
    return 0;
}
