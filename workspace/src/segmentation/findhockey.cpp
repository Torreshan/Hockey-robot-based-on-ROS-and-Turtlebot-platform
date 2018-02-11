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
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
using namespace cv;
using namespace std;
void Erosion( int, void* );
void Dilation( int, void* );
const double camera_fx =570.3422241210938;
const double camera_fy =570.3422241210938;
const double camera_cx =319.5;
const double camera_cy =239.5;
const double camera_factor = 1000;
string color ;
bool kg = false;
int m = 0;
bool s=true;
int num;
//int g = 0;
int l= 0;
int mm = 0;
bool flage = false;
class Facedetection{


    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Publisher bb_pub_;
    ros::Publisher gogo;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber image_depth_sub_;
public:
    Facedetection()
        : it_(nh_)
    {

    bb_pub_ = nh_.advertise<geometry_msgs::Point>("/color_segmentation/bb",1);
    gogo = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",1);

    image_sub_ = it_.subscribe("/camera/rgb/image_rect_color",1,&Facedetection::imagecallback,this);

    //image_depth_sub_ = it_.subscribe("/camera/depth_registered/image",1,&Facedetection::imagecallback1,this);
    //namedWindow("Face");
    cout << "======colorwork======" << endl;

    }
    ~Facedetection()
    {
        //destroyWindow("Face");
        cout << "colorwork" << endl;
    }
    /*Mat cv_im;
    Mat im_gray;
    CascadeClassifier face_cascade;
    std::vector<Rect> faces;
    perception_msgs::Rect r;*/
    //void imagecallback(const sensor_msgs::ImageConstPtr& image);

void imagecallback(const sensor_msgs::ImageConstPtr& image){
   /* try
    {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("CVBRIDGE:{}");
        return;

    }*/


    l++;
    if(flage){
        cout << "colorwork" << endl;

    if(l>20){
    if(color=="blue"||color=="yellow"){
    if (s)
    {



    cv_bridge::CvImagePtr cv_ptr;
    try{

    cv_ptr = cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::BGR8);
        }
    catch(cv_bridge::Exception& e){
    ROS_ERROR("CVBRIDGE:{} %s",e.what());
    return;

        }
    Mat imagee = cv_ptr->image;

    Rect rect(0,0,640,240);
    Mat image_roi = imagee(rect);
    cv::Mat cv_im1;
    cv::Mat cv_im ;


    Mat result0,result;
    vector<Mat> cha;
    //int borderType = BORDER_REPLICATE;
    int top = (int)(20);
    int bottom = (int) (51);
    int left = (int) (20);
    int right = (int) (20);

    int borderType = BORDER_CONSTANT;
    Scalar value = Scalar(0,0,0);
    copyMakeBorder(image_roi,result0,top,bottom,left,right,borderType,value);
    split(result0,cha);
    for(int i=260;i<290;i++){
        for(int j=0; j<690;j++){
            cha[0].at<uchar>(i,j)=0;
            cha[1].at<uchar>(i,j)=0;
            cha[2].at<uchar>(i,j)=0;


        }



    }

      for(int i=0;i<100;i++){
        for(int j=0; j<690;j++){
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
    imshow("a",cv_im);
    cv::waitKey(3);
    //Mat element = getStructuringElement(MORPH_ELLIPSE, Size(20,20));
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
    if (color=="yellow")
    {
        /* code */

    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2,2));
    morphologyEx(cv_im, cv_im, MORPH_OPEN, element);
    morphologyEx(cv_im, cv_im, MORPH_CLOSE, element);
    morphologyEx(im_gray, im_gray, MORPH_OPEN, element);
    morphologyEx(im_gray, im_gray, MORPH_CLOSE, element);
    }
    h_plane.create(im_gray.size(),im_gray.type());
    s_plane.create(im_gray.size(),im_gray.type());
    v_plane.create(im_gray.size(),im_gray.type());
    vector<Mat>hsv_planes;
    split(cv_im, hsv_planes);
    h_plane = hsv_planes[0];
    s_plane = hsv_planes[1];
    v_plane = hsv_planes[2];


    //=========================================================================

        for(int j = 0;j < im_gray.rows;j++){
            const uchar* data_h = h_plane.ptr<uchar>(j);
            const uchar* data_s = s_plane.ptr<uchar>(j);
            const uchar* data_v = v_plane.ptr<uchar>(j);

            for(int i=0; i<im_gray.cols*im_gray.channels(); i++){


//                cout<< "3" << endl;

            if(color=="blue"){



                   if (data_h[i]*2>210&&data_h[i]*2<270  &&  data_s[i]>40  &&  data_v[i]>40){
//                if (220<data_h[i]*2<270 && data_s[i]>40&&data_v[i]>20){
//========================================================================                if (220<data_h[i]*2&&data_h[i]*2<270&&data_s[i]>40&&data_v[i]>20){
                //if (data_h[i]*2>220&&data_h[i]*2<270){
                //if (data_h[i]*2>60&&data_h[i]*2<120&&data_s[i]>20&&data_v[i]>10){
                    im_gray.at<uchar>(j,i)=0;

                }

                else{

                    im_gray.at<uchar>(j,i)=255;

                }
        }


            else if(color=="yellow"){

                       if (data_h[i]*2> 44&&data_h[i]*2<78  &&  data_s[i]>40  &&  data_v[i]>40){

//                   if (45<data_h[i]*2<60    &&  65<data_s[i]*2<100   &&     70<data_v[i]*2<100){





  //              if (45<data_h[i]*2<70    &&  100<data_s[i]  &&     70<data_v[i]*2){




            //=========================================================================             if (52<data_h[i]*2&&data_h[i]*2<68&&data_s[i]>100&&data_v[i]>100){
                    //if (data_h[i]*2>220&&data_h[i]*2<270){
                    //if (data_h[i]*2>60&&data_h[i]*2<120&&data_s[i]>20&&data_v[i]>10){
                        im_gray.at<uchar>(j,i)=0;

                    }

                    else{

                        im_gray.at<uchar>(j,i)=255;

                    }
            }

             else;


            }

        }
     //cout<< "6666666" << endl;
//=============================================================================0
        if (color=="blue")
    {
        Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2,2));
        morphologyEx(im_gray,im_gray,MORPH_OPEN,element);
        morphologyEx(im_gray,im_gray,MORPH_CLOSE,element);
        dilation_dst = im_gray;

        //erode(im_gray, erosion_dst, element);
        //dilate(erosion_dst, dilation_dst, element );
    }
    else if (color == "yellow")
    {
        /*Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2,2));
        morphologyEx(im_gray,im_gray,MORPH_OPEN,element);
        morphologyEx(im_gray,im_gray,MORPH_CLOSE,element);*/
        dilation_dst = im_gray;
    }

    imshow("binary", dilation_dst);
    cv::waitKey(3);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarachy;
    int hockeyNum = 0;


    findContours(dilation_dst,contours,hierarachy,CV_RETR_CCOMP,CV_CHAIN_APPROX_NONE);
    vector<vector<Point> > contours_poly(contours.size());
    vector<Rect> boundRect(contours.size());
    cv::Rect r;
    Mat imagegg = Mat::zeros(dilation_dst.size(),dilation_dst.type());

    int index = 0;
    if (color == "yellow")
    {
        /* code */

    for ( int i=0; i<contours.size(); i++)
    {
        double max = 500;
        approxPolyDP(Mat(contours[i]),contours_poly[i],3,true);
        boundRect[i] = boundingRect(Mat(contours_poly[i]));
        double size = boundRect[i].width*boundRect[i].height;
        if(( size > max)&&(boundRect[i].height>2.0*boundRect[i].width)&&(boundRect[i].height<5*boundRect[i].width))
        {
            max = size;
            index = i;
            hockeyNum ++;
        }
    }

    }
    else{
        for ( int i=0; i<contours.size(); i++)
        {
        double max = 500;
        approxPolyDP(Mat(contours[i]),contours_poly[i],3,true);
        boundRect[i] = boundingRect(Mat(contours_poly[i]));
        double size = boundRect[i].width*boundRect[i].height;
        if(( size > max)&&(boundRect[i].height>2.2*boundRect[i].width)&&(boundRect[i].height<5*boundRect[i].width))
        {
            max = size;
            index = i;
            hockeyNum ++;
        }
        }



                }

    if (hockeyNum == 0){
        cout << "nothing" << endl;
          if ( m <= 7){
                    ros::Duration(3).sleep();
                    geometry_msgs::Twist msg;
                     msg.linear.x= 0;
                     msg.angular.z=0.7;
                     gogo.publish(msg);
                     ros::Duration(3).sleep();

                }
                else if (12>=m > 7){
                    geometry_msgs::Twist msg;
                     msg.linear.x= 0;
                     msg.angular.z=-1;
                     gogo.publish(msg);
                     ros::Duration(3).sleep();


                }
                else if (12 < m <= 14 ){
                    geometry_msgs::Twist msg;
                     msg.linear.x= 0;
                     msg.angular.z=-2;
                     gogo.publish(msg);
                     ros::Duration(3).sleep();


                }
                else if (14 < m  ){
                    geometry_msgs::Twist msg;
                     msg.linear.x= 1;
                     msg.angular.z= 0 ;
                     gogo.publish(msg);
                     ros::Duration(3).sleep();
                     m =0;


                }
                m++;
                }
                geometry_msgs::Point p;
                if(hockeyNum != 0){
                    r.x = boundRect[index].x;
                    r.y = boundRect[index].y;
                    r.width = boundRect[index].width;
                    r.height = boundRect[index].height;
                     //bb_pub_.publish(p);
                      p.x = boundRect[index].x + boundRect[index].width/2;
                      p.y = boundRect[index].y + boundRect[index].height/2;
                    drawContours( imagegg, contours_poly, index, Scalar(255,0,255), 1, 8, vector<Vec4i>(), 0, Point() );
                    rectangle(imagegg,r.tl(),r.br(),Scalar(255,0,255),2);
                    //s = false;

                    if (1<m <= 7){
                    geometry_msgs::Twist msg;
                     msg.linear.x= 0;
                     msg.angular.z=0.7;
                     gogo.publish(msg);
                     ros::Duration(3).sleep();

                }
                else if (12>=m > 7){
                    geometry_msgs::Twist msg;
                     msg.linear.x= 0;
                     msg.angular.z=-1;
                     gogo.publish(msg);
                     ros::Duration(3).sleep();

                }
                else if (10 < m  ){
                    geometry_msgs::Twist msg;
                     msg.linear.x= 0;
                     msg.angular.z=-2;
                     gogo.publish(msg);
                     ros::Duration(3).sleep();

                }
                s = false;
                m = 0;

                imshow("binary1", imagegg);
                cv::waitKey(3);

}
}
}

    if(s==false){


    cv_bridge::CvImagePtr cv_ptr;
    try{

    cv_ptr = cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::BGR8);
        }
    catch(cv_bridge::Exception& e){
    ROS_ERROR("CVBRIDGE:{} %s",e.what());
    return;

        }
    Mat imagee = cv_ptr->image;

    Rect rect(0,0,640,240);
    Mat image_roi = imagee(rect);
    cv::Mat cv_im1;
    cv::Mat cv_im ;


    Mat result0,result;
    vector<Mat> cha;
    //int borderType = BORDER_REPLICATE;
    int top = (int)(20);
    int bottom = (int) (51);
    int left = (int) (20);
    int right = (int) (20);
    int borderType = BORDER_CONSTANT;
    Scalar value = Scalar(0,0,0);
    copyMakeBorder(image_roi,result0,top,bottom,left,right,borderType,value);
    split(result0,cha);
    for(int i=260;i<290;i++){
        for(int j=0; j<690;j++){
            cha[0].at<uchar>(i,j)=0;
            cha[1].at<uchar>(i,j)=0;
            cha[2].at<uchar>(i,j)=0;


        }

        for(int i=0;i<100;i++){
        for(int j=0; j<690;j++){
            cha[0].at<uchar>(i,j)=0;
            cha[1].at<uchar>(i,j)=0;
            cha[2].at<uchar>(i,j)=0;


        }
    }

    }
    merge(cha,result);








    GaussianBlur(result,cv_im1,cv::Size(13,13),0,0);
    Mat kernel = (Mat_<int>(3,3) << 0,-1,0,-1,5,-1,0,-1,0);
    filter2D(cv_im1,cv_im,cv_im1.depth(),kernel);
    cv_im = result;

    imshow("a",cv_im);
    cv::waitKey(3);
    //Mat element = getStructuringElement(MORPH_ELLIPSE, Size(20,20));
    Mat erosion_dst,dilation_dst,im_gray;

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
    /*if (color=="yellow")
    {


    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2,2));
    morphologyEx(cv_im, cv_im, MORPH_OPEN, element);
    morphologyEx(cv_im, cv_im, MORPH_CLOSE, element);
    morphologyEx(im_gray, im_gray, MORPH_OPEN, element);
    morphologyEx(im_gray, im_gray, MORPH_CLOSE, element);
    }*/
    h_plane.create(im_gray.size(),im_gray.type());
    s_plane.create(im_gray.size(),im_gray.type());
    v_plane.create(im_gray.size(),im_gray.type());
    vector<Mat>hsv_planes;
    split(cv_im, hsv_planes);
    h_plane = hsv_planes[0];
    s_plane = hsv_planes[1];
    v_plane = hsv_planes[2];


    //=========================================================================

        for(int j = 0;j < im_gray.rows;j++){
            const uchar* data_h = h_plane.ptr<uchar>(j);
            const uchar* data_s = s_plane.ptr<uchar>(j);
            const uchar* data_v = v_plane.ptr<uchar>(j);

            for(int i=0; i<im_gray.cols*im_gray.channels(); i++){


//                cout<< "3" << endl;

            if(color=="blue"){



                   if (data_h[i]*2>210&&data_h[i]*2<270  &&  data_s[i]>40  &&  data_v[i]>40){
//                if (220<data_h[i]*2<270 && data_s[i]>40&&data_v[i]>20){
//========================================================================                if (220<data_h[i]*2&&data_h[i]*2<270&&data_s[i]>40&&data_v[i]>20){
                //if (data_h[i]*2>220&&data_h[i]*2<270){
                //if (data_h[i]*2>60&&data_h[i]*2<120&&data_s[i]>20&&data_v[i]>10){
                    im_gray.at<uchar>(j,i)=0;

                }

                else{

                    im_gray.at<uchar>(j,i)=255;

                }
        }


            else if(color=="yellow"){

                       if (data_h[i]*2>44&&data_h[i]*2<78  &&  data_s[i]>40  &&  data_v[i]>40){


                        im_gray.at<uchar>(j,i)=0;

                    }

                    else{

                        im_gray.at<uchar>(j,i)=255;

                    }
            }

            else;


            }

        }
     //cout<< "6666666" << endl;
//=============================================================================0

    if (color=="blue")
    {
        Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2,2));
        morphologyEx(im_gray,im_gray,MORPH_OPEN,element);
        morphologyEx(im_gray,im_gray,MORPH_CLOSE,element);
        dilation_dst = im_gray;
        //erode(im_gray, erosion_dst, element);
        //dilate(erosion_dst, dilation_dst, element );
    }
    else if (color == "yellow")
    {
        //erode(im_gray, erosion_dst, element);
        //dilate(erosion_dst, dilation_dst, element );

        /*Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2,2));
        morphologyEx(im_gray,im_gray,MORPH_OPEN,element);
        morphologyEx(im_gray,im_gray,MORPH_CLOSE,element);*/
        dilation_dst = im_gray;
    }
    //erode(im_gray, erosion_dst, element);
    //dilate(erosion_dst, dilation_dst, element );
    //dilation_dst = im_gray;
    //imshow("binary", dilation_dst);
    //cv::waitKey(3);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarachy;
    int hockeyNum = 0;


    findContours(dilation_dst,contours,hierarachy,CV_RETR_CCOMP,CV_CHAIN_APPROX_NONE);
    vector<vector<Point> > contours_poly(contours.size());
    vector<Rect> boundRect(contours.size());
    cv::Rect r;
    Mat imagegg = Mat::zeros(dilation_dst.size(),dilation_dst.type());
    double max = 500;
    int index = 0;
    if (color == "blue"){
    for ( int i=0; i<contours.size(); i++)
    {
        approxPolyDP(Mat(contours[i]),contours_poly[i],3,true);
        boundRect[i] = boundingRect(Mat(contours_poly[i]));
        double size = boundRect[i].width*boundRect[i].height;
        if((size > max)&&(boundRect[i].height>2*boundRect[i].width)&&(boundRect[i].height<6*boundRect[i].width))
        {
            max = size;
            index = i;
            hockeyNum ++;
        }

        //cout << hockeyNum << endl;


                }
    }
    else {

        for ( int i=0; i<contours.size(); i++)
        {
            approxPolyDP(Mat(contours[i]),contours_poly[i],3,true);
            boundRect[i] = boundingRect(Mat(contours_poly[i]));
            double size = boundRect[i].width*boundRect[i].height;
            if((size > max)&&(boundRect[i].height>2*boundRect[i].width)&&(boundRect[i].height<6*boundRect[i].width))
            {
                max = size;
                index = i;
                hockeyNum ++;
            }

            //cout << hockeyNum << endl;


                    }

    }

    if(hockeyNum != 0) num++;
    if (hockeyNum == 0){
        cout << "nothing" << endl;
        mm++;

    }
    if(mm > 10)
    {
        s=true;
        mm= 0;

    }



        geometry_msgs::Point p;
                if(num > 10){
                    r.x = boundRect[index].x;
                    r.y = boundRect[index].y;
                    r.width = boundRect[index].width;
                    r.height = boundRect[index].height;
                     //bb_pub_.publish(p);
                      p.x = boundRect[index].x + boundRect[index].width/2;
                      p.y = boundRect[index].y + boundRect[index].height/2;
                      bb_pub_.publish(p);
                    drawContours( imagegg, contours_poly, index, Scalar(255,0,255), 1, 8, vector<Vec4i>(), 0, Point() );
                    rectangle(imagegg,r.tl(),r.br(),Scalar(255,0,255),2);
                    num = 0;


}
                //imshow("binary1", imagegg);
                //cv::waitKey(3);









            }
        }
}
}
//    double area;
//    double aa=0;
//    int b = 0;


//     cout<< "7" << endl;
//    for (int i=0; i<(int)contours.size();i++){
//        area = contourArea(contours[i],true);
//        if(area>aa){
//            aa = area;
//            b = i;
//        }

//    }

//     cout<< "8" << endl;


//     if(contours.size()>0){
//         vector<Moments> mu(1);
//         mu[0] = moments(contours[b],false);
//         vector<Point2f> mc(1);
//         mc[0] = Point2d(mu[0].m10/mu[0].m00 , mu[0].m01/mu[0].m00);
//         cout << mc[0].x <<endl;
//         Mat imagegg = Mat::zeros(dilation_dst.size(),dilation_dst.type());


//         cout<< "99999999" << endl;


//         //    if((r.x+r.width)<640&&(r.x+r.width)<480){
//         //    bb_pub_.publish(r);
//         //}
//         //}
//        //cout<<boundRect[b].width<<endl;
//         /*vector<vector<Point> > contours_poly(contours.size());
//         vector<Rect> boundRect(contours.size());
//         perception_msgs::Rect r;
//         Mat imagegg = Mat::zeros(dilation_dst.size(),dilation_dst.type());
//         for ( int i=0; i<contours.size(); i++){
//             approxPolyDP(Mat(contours[i]),contours_poly[i],3,true);
//             boundRect[i] = boundingRect(Mat(contours_poly[i]));






//         }
//         int a;
//         int b= 0;
//         int k = 0 ;

//         int c = boundRect[k].width*boundRect[k].height;
//         c = 0;
//         if (c > 107200){
//             k=1;
//             c= boundRect[k].width*boundRect[k].height;
//         }
//         for ( int i=0;i<contours.size(); i++){
//           //if( 4*boundRect[i].width > boundRect[i].height > 2*boundRect[i].width)
//             if(  boundRect[i].height < 4*boundRect[i].width)
//             continue;
//         if((boundRect[i].width*boundRect[i].height) < 5000)
//             continue;
//           a =boundRect[i].width*boundRect[i].height;
//           //cout<< "a" << a << "c" << c << endl;
//          if (c<a){
//              c = a;
//              b = i;

//          }







//         }
//         //cout<< "b" << b << endl;
//         //for(int b=0;b<contours.size(); b++){
//             r.x = boundRect[b].x;
//             r.y = boundRect[b].y;
//             r.width = boundRect[b].width;
//             r.height = boundRect[b].height;
//             //rectangle(im_gray,cv::Point(boundRect[i].x,boundRect[i].y),cv::Point(boundRect[i].x+boundRect[i].width,boundRect[i].y+boundRect[i].height),Scalar(0,255,0),2);
//             drawContours( imagegg, contours_poly, b, Scalar(255,0,255), 1, 8, vector<Vec4i>(), 0, Point() );
//             rectangle(imagegg,boundRect[b].tl(),boundRect[b].br(),Scalar(255,0,255),2);
//             */
//            // if((r.x+r.width)<640&&(r.x+r.width)<480){
//             //bb_pub_.publish(r);
//         //}
//         //}
//        //cout<<boundRect[b].width<<endl;
//     //    for ( int i=0; i<contours.size(); i++){

//     //        r.x = boundRect[i].x;
//     //        r.y = boundRect[i].y;
//     //        r.width = boundRect[i].width;
//     //        r.height = boundRect[i].height;
//     //        //rectangle(im_gray,cv::Point(boundRect[i].x,boundRect[i].y),cv::Point(boundRect[i].x+boundRect[i].width,boundRect[i].y+boundRect[i].height),Scalar(0,255,0),2);
//          //cout<< "9" << endl;
//             //drawContours( imagegg, contours_poly, i, Scalar(255,0,255), 1, 8, vector<Vec4i>(), 0, Point() );
//             for(int i = 0; i<contours.size();i++){
//                 drawContours( imagegg, contours, i, Scalar(255,0,255), 1, 8, vector<Vec4i>(), 0, Point() );


//             }
//             circle(imagegg, mc[0], 5, Scalar(255,255,255), -1, 8, 0);
//            geometry_msgs::Point p;
//            p.x = mc[0].x;
//            p.y = mc[0].y;

//             bb_pub_.publish(p);

//              cout<< "10" << endl;

//             //rectangle(imagegg,boundRect[i].tl(),boundRect[i].br(),Scalar(255,0,255),2);
//     //        bb_pub_.publish(r);





//     //    }



//         imshow("binary1", imagegg);
//         cv::waitKey(3);


};





    bool main_(std_srvs::SetBool::Request &req1,std_srvs::SetBool::Response &res1){
        s=req1.data;
        if(s==true){
            color="yellow";
        }
        else{
            color="blue";
        }
        res1.message=color;


    cout<< color << endl;
    return true;
}

bool start_to_color(std_srvs::SetBool::Request &req1,std_srvs::SetBool::Response &res1){

    flage = true;
    if(req1.data==true){
            color="yellow";
        }
        else{
            color="blue";
        }
 return true;

}
bool finish_to_color(std_srvs::Empty::Request &req1,std_srvs::Empty::Response &res1){

    flage = false;
    destroyAllWindows();

 return true;

}


int main(int argc, char** argv){
    //Facedetection::imagecallback &imagedeal;
    ros::init(argc,argv,"color_segmentation");
    ros::NodeHandle n;
    //ros::Subscriber image_sub = n.subscribe("/kinect2/hd/image_color",&Facedetection::imagecallback);
    //ros::Publisher bb_pub = n.advertise<perception_msgs::Rect>("face_detection/bb",1);
    //perception_msgs::Rect r;
    //r = imagedeal.r;
    //bb_pub.publish(r);

//    ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>("color_detection");

      ros::ServiceServer service = n.advertiseService("color_segmentation",main_);
      ros::ServiceServer service1 = n.advertiseService("start_to_color",start_to_color);
      ros::ServiceServer service2 = n.advertiseService("finish_to_color",finish_to_color);
//    std_srvs::SetBool srv;
//    srv.request.data = true;
//     << kg << endl;
//    if(kg){
//    if(client.call(srv))
//    {
//        cout<< srv.response.message<< endl;
//        color = srv.response.message;
//        Facedetection ic;
//        ros::spinOnce();
//    }
//    else
//    {
//        ROS_ERROR("Failed to call service color_detection");
//        return 1;
//    }
//}


    Facedetection ic;
    ros::spin();



    return 0;
}
