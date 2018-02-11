#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <set>
#include <iterator>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <cmath>
#include <std_srvs/Empty.h>
#include "hockeysearch/pg_info_transfer.h"

ros::Publisher aim_pub;
using namespace cv;
using namespace std;
ros::ServiceClient stop_clt;
int forward = 5;
double speed = 0.6;
double searchspeed = 0.6;
//int hockeyNum = 0;
bool isForward = false;
//static int hockeyNum = 0;
bool catch_state = false;
geometry_msgs::Twist msg;
bool workFlag = false;
int times = 0;
int color = 1;

int searchtimes = 0;


//class hockeysearch{
////    ros::NodeHandle nh_;
////    image_transport::ImageTransport it_;
////    //ros::Publisher bb_pub_;
////    ros::Publisher aim_pub;
////    ros::ServiceServer state_pub;
////    image_transport::Subscriber image_sub_;

//    //image_transport::Subscriber image_depth_sub_;
//   // ros::Publisher odom_pub;
//   // tf::TransformBroadcaster odom_broadcaster;

//public:
//   // int flag ;

//    int forward;
//    hockeysearch()
// //       : it_(nh_)
//    {
////        aim_pub = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
////        state_pub = nh_.advertiseService<std_srvs::Empty>("catching_state",&hockeysearch::callback,this);
////        image_sub_ = it_.subscribe("/camera/rgb/image_raw",1,&hockeysearch::imagecallback,this);
//       // odom_pub = nh_.advertise<nav_msgs::Odometry>("odom", 50);
//        namedWindow("hockey");
//       // ros::Time current_time, last_time;
//       // current_time = ros::Time::now();
//       // last_time = ros::Time::now();


//    }
//    ~hockeysearch()
//    {
//        destroyWindow("hockey");
//    }
//*************************when no cones, turn to search**********************************************//
	void aTurn(double turnAngle)
    {
        if(workFlag){
        msg.linear.x = 0.0;
        msg.angular.z = -0.6;
        aim_pub.publish(msg);
        ros::Duration(0.15).sleep();
        }

    }
//*********************** turn**********************************************************************//    
	void Turn(double turnAngle)
    {
        if(workFlag){
        msg.linear.x = 0.1;
        msg.angular.z = 0.4;
        aim_pub.publish(msg);
        ros::Duration(0.15).sleep();
        }

    }
//*************************anti turn***********************************************************//
    void antiTurn(double turnAngle)
    {
        if(workFlag){
        msg.linear.x = 0.1;
        msg.angular.z = -0.4;
        aim_pub.publish(msg);
        ros::Duration(0.15).sleep();
        }

    }
//***********************when adjust to one cone, go straight to catch**************************//    
    void Moveforward()
    {
        if(workFlag){
        msg.linear.x = 0.3;
        msg.angular.z = 0.01;
        aim_pub.publish(msg);
        ros::Duration(0.1).sleep();
        }

    }

    bool pg_info_callback(hockeysearch::pg_info_transfer::Request &req,
                          hockeysearch::pg_info_transfer::Response &res){

        color = req.color; //0:blue   1:yellow
	    res.success = true;
        return true;
    }

    bool starting (std_srvs::Empty::Request& e,std_srvs::Empty::Response& ee)
    {
        workFlag = !workFlag;
        return true;
    }

    void imagecallback(const sensor_msgs::ImageConstPtr& image)
    {
        if(workFlag){
            times++;
            if (times>20)
            {
                /* code */

            if(color == 0){
//***************************aim color is blue**************************************************//                
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("CVBRIDGE:{} %s",e.what());
            return;

        }
	


        Mat imageraw = cv_ptr->image;

        Rect rect(0,0,640,240);
        Mat image_roi = imageraw(rect);
        Mat cv_im,cv_im1;
        Mat result0,result;
        vector<Mat> cha;
        int top = (int)(20);
        int bottom = (int) (51);
        int left = (int) (80);
        int right = (int) (80);
        int borderType = BORDER_CONSTANT;
        Scalar value = Scalar(0,0,0);
        copyMakeBorder(image_roi,result0,top,bottom,left,right,borderType,value);
        split(result0,cha);


        for(int i=0;i<30;i++)
        {
            for(int j=0; j<690;j++)
            {
                cha[0].at<uchar>(i,j)=0;
                cha[1].at<uchar>(i,j)=0;
                cha[2].at<uchar>(i,j)=0;
            }
        }

        for(int i=240;i<290;i++)
        {
            for(int j=0; j<720;j++)
            {
                cha[0].at<uchar>(i,j)=0;
                cha[1].at<uchar>(i,j)=0;
                cha[2].at<uchar>(i,j)=0;
            }
        }

        merge(cha,result);
        GaussianBlur(result,cv_im1,cv::Size(13,13),0,0);
        Mat kernel = (Mat_<int>(3,3) << 0,-1,0,-1,5,-1,0,-1,0);
        filter2D(cv_im1,cv_im,cv_im1.depth(),kernel);
        cv_im = result;
        //imshow("a",cv_im);
        //cv::waitKey(3);
        //Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2,2));
        Mat erosion_dst,dilation_dst,im_gray;
        //erode(cv_im, erosion_dst, element);
        //dilate(erosion_dst, dilation_dst, element );
        Mat h_plane;
        Mat s_plane;
        Mat v_plane;
        Mat dst;
        cvtColor(cv_im,cv_im, CV_BGR2HSV);
        Mat tempImg[3];
        split(cv_im, tempImg);
        equalizeHist(tempImg[2], tempImg[2]);
        cv::merge(tempImg, 3 , cv_im);
        cvtColor(cv_im, im_gray,CV_RGB2GRAY);

        h_plane.create(im_gray.size(),im_gray.type());
        s_plane.create(im_gray.size(),im_gray.type());
        v_plane.create(im_gray.size(),im_gray.type());
        vector<Mat>hsv_planes;
        split(cv_im, hsv_planes);
        h_plane = hsv_planes[0];
        s_plane = hsv_planes[1];
        v_plane = hsv_planes[2];
        //int rows = im_gray.rows;
        int columns = im_gray.cols;
        int rows = im_gray.rows;
        double area = 240 * (rows -100) ;
        int keypoint = 0;



        for(int j = 0;j < rows;j++)
        {
            const uchar* data_h = h_plane.ptr<uchar>(j);
            const uchar* data_s = s_plane.ptr<uchar>(j);
            const uchar* data_v = v_plane.ptr<uchar>(j);
            for(int i=0; i<columns; i++)
            {

                //if (data_h[i]*2> 44&&data_h[i]*2<78  &&  data_s[i]>40  &&  data_v[i]>40){ //yellow
               if (data_h[i]*2>180&&data_h[i]*2<270  &&  data_s[i]>40  &&  data_v[i]>40){ //blue
                //if (data_h[i]*2>60&&int hockeyNum = 0;data_h[i]*2<120&&data_s[i]>20&&data_v[i]>10){
                    im_gray.at<uchar>(j,i)=0;
                    if(i>370&&i<640)
                    keypoint ++;
                }

                else
                {
                   im_gray.at<uchar>(j,i)=255;
                    

                }

            }

        }
        if(keypoint/area>0.5){
            ROS_INFO("i get it");
            msg.linear.x = 0.4;
            msg.angular.z = 0;
            aim_pub.publish(msg);
            ros::Duration(0.4).sleep();
            msg.linear.x = 0.0;
            msg.angular.z = 0;
            std_srvs::Empty e;
            stop_clt.call(e);

              catch_state = false;
              workFlag = false;
              destroyAllWindows();

            //cout<<"debug"<<keypoint/area<<endl;
        }



        //erode(im_gray, erosion_dst, element);
        //dilate(erosion_dst, dilation_dst, element );
        Mat element = getStructuringElement(MORPH_ELLIPSE, Size(3,3));
        morphologyEx(im_gray,im_gray,MORPH_OPEN,element);
        morphologyEx(im_gray,im_gray,MORPH_CLOSE,element);
        dilation_dst = im_gray;
        //imshow("binary", dilation_dst);
        //cv::waitKey(3);
        vector<vector<Point> > contours;
        vector<Vec4i> hierarachy;



        findContours(dilation_dst,contours,hierarachy,CV_RETR_CCOMP,CV_CHAIN_APPROX_NONE);
        vector<vector<Point> > contours_poly(contours.size());
        vector<Rect> boundRect(contours.size());
        //vector<Rect> rightRect;
        //Rect rightBound;
        //perception_msgs::Rect r;
        cv::Rect r;

        Mat imagegg = Mat::zeros(dilation_dst.size(),dilation_dst.type());
        double max = 500;
        

        int index = 0;
        int hockeyNum = 0;
        for ( int i=0; i<contours.size(); i++)
        {
            approxPolyDP(Mat(contours[i]),contours_poly[i],3,true);
            boundRect[i] = boundingRect(Mat(contours_poly[i]));
            double size = boundRect[i].width*boundRect[i].height;
            if(( size > max)&&(boundRect[i].height>1.5*boundRect[i].width)&&(boundRect[i].height<6*boundRect[i].width))
            {
                if(size > 100000)
                    continue;
                max = size;
                index = i;
                hockeyNum ++;
            }

         }
        //cout<<"debug hockeynum"<<hockeyNum<<endl;


                    if(hockeyNum == 0)
                    {
                        forward = 2;
                        //double try = 0.1;
                        aTurn(searchspeed);
                        //aim_pub.publish(msg);
                        //ros::Duration().sleep();
                        ROS_INFO_STREAM("=========Turn to seach ball===========");
                        searchtimes++;
                        if(searchtimes > 80){
                            catch_state = true;

                        }



                    }


                   else
                    {
//                        if(hockeyNum >1)
//                        {
//                            index = 0;
//                        }

                        r.x = boundRect[index].x;
                        r.y = boundRect[index].y;
                        r.width = boundRect[index].width;
                        r.height = boundRect[index].height;
                        searchtimes = 0;


                        //rectangle(im_gray,cv::Point(boundRect[i].x,boundRect[i].y),cv::Point(boundRect[i].x+boundRect[i].width,boundRect[i].y+boundRect[i].height),Scalar(0,255,0),2);
                        drawContours( imagegg, contours_poly, index, Scalar(255,0,255), 1, 8, vector<Vec4i>(), 0, Point() );
                        rectangle(imagegg,r.tl(),r.br(),Scalar(255,0,255),2);
                       // bb_pub_.publish(r);




                    if(abs(r.tl().x + 0.5*r.width-0.5*columns)>60 && !isForward)
                    {
                        if(abs(r.tl().x + 0.5*r.width-1.0) > abs(r.tl().x + 0.5*r.width-columns))

                        {
                        forward = 0;
                        antiTurn(speed);

                        //    Moveforward();
                        ROS_INFO_STREAM("==========antiTurn ==========");
                        }

                        else
                        {
                         forward = 2;
                         Turn(speed);

                        //    Moveforward();
                         ROS_INFO_STREAM("==========Turn ==========");
                        }


                        
                    }
                    else

                    {
                        forward = 1;
                        //isForward = true;

                        Moveforward();

                        ROS_INFO_STREAM("=========Moveforward===========");
                    }
                   }
                    //imshow("binary1", imagegg);
                    //cv::waitKey(3);


                }


            else if(color == 1)
//**************************aim color is yellow **************************************//            
            {  // cout<<"i'm yellow"<<endl;
                cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::BGR8);
            }
            catch(cv_bridge::Exception& e)
            {
                ROS_ERROR("CVBRIDGE:{} %s",e.what());
                return;
                }

            


            Mat imageraw = cv_ptr->image;

            Rect rect(0,0,640,240);
            Mat image_roi = imageraw(rect);
            Mat cv_im,cv_im1;
            Mat result0,result;
            vector<Mat> cha;
            //int borderType = BORDER_REPLICATE;
            int top = (int)(20);
            int bottom = (int) (51);
            int left = (int) (80);
            int right = (int) (80);
            int borderType = BORDER_CONSTANT;
            Scalar value = Scalar(0,0,0);
            copyMakeBorder(image_roi,result0,top,bottom,left,right,borderType,value);
            split(result0,cha);

            for(int i=0;i<30;i++)
            {
                for(int j=0; j<690;j++)
                {
                    cha[0].at<uchar>(i,j)=0;
                    cha[1].at<uchar>(i,j)=0;
                    cha[2].at<uchar>(i,j)=0;
                }
            }

            for(int i=240;i<290;i++)
            {
                for(int j=0; j<720;j++)
                {
                    cha[0].at<uchar>(i,j)=0;
                    cha[1].at<uchar>(i,j)=0;
                    cha[2].at<uchar>(i,j)=0;
                }
            }

            merge(cha,result);
            GaussianBlur(result,cv_im1,cv::Size(13,13),0,0);
            Mat kernel = (Mat_<int>(3,3) << 0,-1,0,-1,5,-1,0,-1,0);
            filter2D(cv_im1,cv_im,cv_im1.depth(),kernel);
            cv_im = result;
            //imshow("a",cv_im);
            //cv::waitKey(3);
            //cout << 1 << endl;
            //Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2,2));
            Mat erosion_dst,dilation_dst,im_gray;
            //erode(cv_im, erosion_dst, element);
            //dilate(erosion_dst, dilation_dst, element );
            Mat h_plane;
            Mat s_plane;
            Mat v_plane;
            Mat dst;
            cvtColor(cv_im,cv_im, CV_BGR2HSV);


            Mat tempImg[3];
            split(cv_im, tempImg);
            equalizeHist(tempImg[2], tempImg[2]);

            cvtColor(cv_im, im_gray,CV_RGB2GRAY);
            Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2,2));
            morphologyEx(cv_im, cv_im, MORPH_OPEN, element);
            morphologyEx(cv_im, cv_im, MORPH_CLOSE, element);
            morphologyEx(im_gray, im_gray, MORPH_OPEN, element);
            morphologyEx(im_gray, im_gray, MORPH_CLOSE, element);

            h_plane.create(im_gray.size(),im_gray.type());
            s_plane.create(im_gray.size(),im_gray.type());
            v_plane.create(im_gray.size(),im_gray.type());
            vector<Mat>hsv_planes;
            split(cv_im, hsv_planes);
            h_plane = hsv_planes[0];
            s_plane = hsv_planes[1];
            v_plane = hsv_planes[2];
            //int rows = im_gray.rows;
            int columns = im_gray.cols;
            int rows = im_gray.rows;
            double area = 240 * (rows-100);
            int keypoint = 0;



            for(int j = 0;j < rows;j++)
            {
                const uchar* data_h = h_plane.ptr<uchar>(j);
                const uchar* data_s = s_plane.ptr<uchar>(j);
                const uchar* data_v = v_plane.ptr<uchar>(j);
                for(int i=0; i<columns; i++)
                {

                    if (data_h[i]*2> 44&&data_h[i]*2<78  &&  data_s[i]>40  &&  data_v[i]>40){ //yellow
                   // {
                   //if (data_h[i]*2>210&&data_h[i]*2<270  &&  data_s[i]>40  &&  data_v[i]>40){ //blue
                    //if (data_h[i]*2>60&&int hockeyNum = 0;data_h[i]*2<120&&data_s[i]>20&&data_v[i]>10){
                        im_gray.at<uchar>(j,i)=0;
                        if(i>370&&i<640)
                        keypoint ++;

                    }

                    else
                    {

                        im_gray.at<uchar>(j,i)=255;
                        //background_num ++;

                    }




                }



            }
    //cout << keypoint << endl;
    //cout <<"debug"<<keypoint/area << endl;

            if((keypoint/area)>(0.55)){
                ROS_INFO("i get it");
                msg.linear.x = 0.4;
                msg.angular.z = 0;
                aim_pub.publish(msg);
                ros::Duration(0.4).sleep();
                msg.linear.x = 0.0;
                msg.angular.z = 0;
                catch_state = true;
                std_srvs::Empty e;
                stop_clt.call(e);

                  catch_state = false;
                  workFlag = false;
                  destroyAllWindows();




           
            }




            //erode(im_gray, erosion_dst, element);
            //dilate(erosion_dst, dilation_dst, element );
		    dilation_dst = im_gray;
            //imshow("binary", dilation_dst);
            //cv::waitKey(3);
            vector<vector<Point> > contours;
            vector<Vec4i> hierarachy;



            findContours(dilation_dst,contours,hierarachy,CV_RETR_CCOMP,CV_CHAIN_APPROX_NONE);
            vector<vector<Point> > contours_poly(contours.size());
            vector<Rect> boundRect(contours.size());
            //vector<Rect> rightRect;
            //Rect rightBound;
            //perception_msgs::Rect r;
            cv::Rect r;

            Mat imagegg = Mat::zeros(dilation_dst.size(),dilation_dst.type());




            double max = 2000;
            //vector<int> index(3);

            int index = 0;
            int hockeyNum = 0;
            for ( int i=0; i<contours.size(); i++)
            {
                approxPolyDP(Mat(contours[i]),contours_poly[i],3,true);
                boundRect[i] = boundingRect(Mat(contours_poly[i]));
                double size = boundRect[i].width*boundRect[i].height;
                if(( size > max)&&(boundRect[i].height>2.2*boundRect[i].width)&&(boundRect[i].height<6*boundRect[i].width))
                {
                    if(size > 100000)
                        continue;
                    max = size;
                    //index.push_bach(i);
                    index = i;
                    hockeyNum ++;
                }

             }
         //   cout<<"hockeynum"<<hockeyNum<<endl;







//            double max = 2000;
//            int index = 0;
//            int hockeyNum = 0;

//            for ( int i=0; i<contours.size(); i++)
//            {
//                approxPolyDP(Mat(contours[i]),contours_poly[i],3,true);
//                boundRect[i] = boundingRect(Mat(contours_poly[i]));
//                double size = boundRect[i].width*boundRect[i].height;
//                if(( size > max)&&(boundRect[i].height>2.5*boundRect[i].width)&&(boundRect[i].height<5*boundRect[i].width))
//                {
//                    max = size;
//                    index = i;
//                    hockeyNum ++;
//                }




//             }

                        if(hockeyNum == 0)
                        {
                            forward = 2;
                            //double try = 0.1;
                            aTurn(searchspeed);
                            //ros::Duration().sleep();
                            //aim_pub.publish(msg);
                            ROS_INFO_STREAM("=========Turn to seach ball===========");

                        }

                       else {
                            r.x = boundRect[index].x;
                            r.y = boundRect[index].y;
                            r.width = boundRect[index].width;
                            r.height = boundRect[index].height;

                            //searchtimes = 0;
                            //rectangle(im_gray,cv::Point(boundRect[i].x,boundRect[i].y),cv::Point(boundRect[i].x+boundRect[i].width,boundRect[i].y+boundRect[i].height),Scalar(0,255,0),2);
                            drawContours( imagegg, contours_poly, index, Scalar(255,0,255), 1, 8, vector<Vec4i>(), 0, Point() );
                            rectangle(imagegg,r.tl(),r.br(),Scalar(255,0,255),2);
                           // bb_pub_.publish(r);

                        if(abs(r.tl().x + 0.5*r.width-0.5*columns)>60 && !isForward)
                        {
                            if(abs(r.tl().x + 0.5*r.width-1.0) > abs(r.tl().x + 0.5*r.width-columns))

                            {
                            forward = 0;
                            antiTurn(speed);
                           // aim_pub.publish(msg);
                            ROS_INFO_STREAM("==========antiTurn ==========");
                            }

                            else
                            {
                             forward = 2;
                             Turn(speed);
                            // aim_pub.publish(msg);
                             ROS_INFO_STREAM("==========Turn ==========");
                            }


                            
                        }
                        else

                        {
                            forward = 1;
                            //isForward = true;

                            Moveforward();
                          //  aim_pub.publish(msg);
                            ROS_INFO_STREAM("=========Moveforward===========");
                        }
                       }

                        


                    }
           }
     }

  }

                int main(int argc, char** argv)
                {
                   // bool run = false;
                    ros::init(argc,argv,"color_segmentation");
                    ros::NodeHandle nh_;
                    image_transport::ImageTransport it_(nh_);
                    //ros::Publisher aim_pub;
                    ros::ServiceServer pg_info_srv;
                    ros::ServiceServer state_pub;

                    ros::ServiceClient fail_clt;
                    //ros::ServiceServer state_pub;
                   // ros::Subscriber laser_sub;
                    image_transport::Subscriber image_sub_;
                    //namedWindow("hockey");

                    pg_info_srv = nh_.advertiseService("catch_pg_info",pg_info_callback);
                    aim_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
                    //color_clt = nh_.advertiseService("catching_state",starting);
                    state_pub = nh_.advertiseService("catching_state",starting);
                    //state_finish = nh_.advertiseService("catching_state",starting);
                    stop_clt = nh_.serviceClient<std_srvs::Empty>("catch_finished");
                    fail_clt = nh_.serviceClient<std_srvs::Empty>("catch_failed");
                    image_sub_ = it_.subscribe("/camera/rgb/image_rect_color",1,imagecallback);


                   // laser_sub = nh_.advertise("/lidarscan",lasercallback);
                    ROS_INFO_STREAM("========== start ==========");
                    ros::Rate rate(10);
                          while(ros::ok())
                         {
//                             if(workFlag == false)
//                                  break;


//                                    catch_state = false;
//                                    workFlag = false;
//                                    destroyAllWindows();
//                                  }

                              //if(isForward)
                              //{
                                  //run = true;
                              //    msg.linear.x = 0.1;
                              //    msg.angular.z = 0;
                                 // image_sub = Null;

                              //}

                              //aim_pub.publish(msg);
                              ros::spinOnce();
                              rate.sleep();
                         }






                    return 0;
                }
