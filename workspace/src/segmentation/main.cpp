#include <ros/ros.h>
#include  <std_srvs/Empty.h>
#include  <geometry_msgs/Twist.h>
#include <std_srvs/SetBool.h>
#include "main_state_machine/pg_info_transfer.h"
using namespace std;
int color;
bool ff = false;
bool flage = false;
bool to_main(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res){

        

      flage = true;
        
    return true;
        
    }
bool getcolor(std_srvs::SetBool::Request &req,std_srvs::SetBool::Response &res){

     if(req.data==true)
            color = 1;
     else
         color = 0;

      ff = true;

    return true;

    }


int main(int argc, char** argv){
   ros::init(argc, argv, "main");
   ros::NodeHandle n;
   bool san=false;
   //string color;
   //cout << 1 << endl;
    //ros::Subscriber image_sub = n.subscribe("/kinect2/hd/image_color",&Facedetection::imagecallback);
    //ros::Publisher bb_pub = n.advertise<perception_msgs::Rect>("face_detection/bb",1);
    //perception_msgs::Rect r;
    //r = imagedeal.r;
    //bb_pub.publish(r);
    cout << "playground_start"<< endl;


    ros::Rate rate(1.5);
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);
    ros::ServiceClient client1 = n.serviceClient<std_srvs::Empty>("playground");
    //ros::ServiceClient client1 = n.serviceClient<std_srvs::Empty>("color", 10);
    ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>("color_detection");
    ros::ServiceClient client2 = n.serviceClient<std_srvs::Empty>("start");
    ros::ServiceServer service = n.advertiseService("start_measure_color",to_main);
    ros::ServiceServer service1 = n.advertiseService("get_the_color",getcolor);


    while(ros::ok()){

      if(flage)
        break;
      //rate.sleep();
      ros::spinOnce();
      rate.sleep();


    }






geometry_msgs::Twist msg2;
  rate.sleep();
   for(int i=0;i<4;i++){
    msg2.linear.x=0;
    msg2.angular.z=1;
    
    //ros::Duration(1);
    vel_pub.publish(msg2);
    rate.sleep();
    ros::spinOnce();
}

  rate.sleep();


  
   for(int i=0;i<6;i++){
    msg2.linear.x=0.37;
    msg2.angular.z=0;
    
    //ros::Duration(1);
    vel_pub.publish(msg2);
    rate.sleep();
    ros::spinOnce();
}
    rate.sleep();
   for(int i=0;i<8;i++){
    msg2.linear.x=0;
    msg2.angular.z=1;
    
    //ros::Duration(1);
    vel_pub.publish(msg2);
    rate.sleep();
    ros::spinOnce();
}
















   rate.sleep();


      std_srvs::Empty sssss;
   //srv_color.request.data=san;

       if(client2.call(sssss))
       {
           //cout<< srv.response.message<< endl;
     //      string color = srv.response.message;
           //Facedetection ic;
           //std::cout << color << std::endl;
           ros::spinOnce();
       }
       else
       {
           ROS_ERROR("Failed to call service color_segmentation");
           return 1;
       }





   //================================ color_segmentation====================










    rate.sleep();
    rate.sleep();
    rate.sleep();
    rate.sleep();
    rate.sleep();




    while(ros::ok()){

      if(ff)
        break;
      //rate.sleep();
      ros::spinOnce();
      rate.sleep();


    }


    rate.sleep();



     for(int i=0;i<6;i++){
      msg2.linear.x=0.37;
      msg2.angular.z=0;

      //ros::Duration(1);
      vel_pub.publish(msg2);
      rate.sleep();
      ros::spinOnce();
  }
      rate.sleep();
     for(int i=0;i<4;i++){
      msg2.linear.x=0;
      msg2.angular.z=1;

      //ros::Duration(1);
      vel_pub.publish(msg2);
      rate.sleep();
      ros::spinOnce();
  }
















     rate.sleep();
   // std_srvs::Empty   srv1;
    //   if(client1.call(srv1)){
     //  ROS_INFO("color");
     //  }
      // else{
       //  ROS_ERROR("Failed");
      // }





//    geometry_msgs::Twist msg3;

//    for(int i=0;i<2;i++){
//     msg3.linear.x=0;
//     msg3.angular.z=1.5;

//     //ros::Duration(1);
//     vel_pub.publish(msg3);
//     rate.sleep();
//     ros::spinOnce();
// }


//    ros::ServiceClient client2 = n.serviceClient<std_srvs::SetBool>("color_segmentation");
//    std_srvs::SetBool srv_color;

//    if(color=="yellow"){
//        san=true;
//    }
//    else{
//        san=false;
//    }

//   srv_color.request.data=san;

//    if(client2.call(srv_color))
//    {
//        //cout<< srv.response.message<< endl;
//  //      string color = srv.response.message;
//        //Facedetection ic;
//        std::cout << color << std::endl;
//        ros::spinOnce();
//    }
//    else
//    {
//        ROS_ERROR("Failed to call service color_segmentation");
//        return 1;
//    }

    ros::ServiceClient client3 = 	  n.serviceClient<main_state_machine::pg_info_transfer>("color_ident_finished");

    main_state_machine::pg_info_transfer srvss;


    if(color==1){
        srvss.request.color = 1;
    }
    else{
        srvss.request.color = 0;
    }
    if(client3.call(srvss))
    {
        //cout<< srv.response.message<< endl;
  //      string color = srv.response.message;
        //Facedetection ic;
        std::cout << color << std::endl;
        ros::spinOnce();
    }
    else
    {
        ROS_ERROR("Failed to call service color_segmentation");
        return 1;
    }


    //=====================================END=================================





   /* geometry_msgs::Twist msg2;
   for(int i=0;i<1;i++){
    msg2.linear.x=0.45;
    msg2.angular.z=0;
    
    //ros::Duration(1);
    vel_pub.publish(msg2);
    rate.sleep();
    ros::spinOnce();
}
*/
    /*std_srvs::Empty   srv;
    if(client.call(srv)){
    ROS_INFO("playground identification");
  	}
 	else{
      ROS_ERROR("Failed");

  	}


 	std_srvs::Empty   srv1;
    if(client1.call(srv1)){
    ROS_INFO("color");
  	}
 	else{
      ROS_ERROR("Failed");

  	}*/

    
  	ros::spin();
  }
