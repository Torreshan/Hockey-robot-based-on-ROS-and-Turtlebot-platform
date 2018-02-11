#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <perception_msgs/Rect.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <vector>
using namespace cv;
using namespace std;
void Erosion( int, void* );
void Dilation( int, void* );
const double camera_fx =570.3422241210938;
const double camera_fy =570.3422241210938;
const double camera_cx =319.5;
const double camera_cy =239.5;
const double camera_factor = 1000;
class Facedetection{


    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Publisher bb_pub_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber image_depth_sub_;
public:
    Facedetection()
        : it_(nh_)
    {

    bb_pub_ = nh_.advertise<perception_msgs::Rect>("/color_segmentation/bb",1);

    image_sub_ = it_.subscribe("/camera/rgb/image_raw",1,&Facedetection::imagecallback,this);

    //image_depth_sub_ = it_.subscribe("/camera/depth/image",1,&Facedetection::imagecallback1,this);
    namedWindow("Face");

    }
    ~Facedetection()
    {
        destroyWindow("Face");
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



    cv_bridge::CvImagePtr cv_ptr;
    try{

    cv_ptr = cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::BGR8);
        }
    catch(cv_bridge::Exception& e){
    ROS_ERROR("CVBRIDGE:{} %s",e.what());
    return;

        }
    cv::Mat cv_im;
    resize(cv_ptr->image,cv_im,cv::Size(),1,1);
    //cv::Mat im_gray;
    //cvtColor(cv_ptr->image,im_gray, CV_BGR2GRAY);
    imshow("a",cv_ptr->image);
    cv::waitKey(3);
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(20,20));
    Mat erosion_dst,dilation_dst,im_gray;
    //erode(cv_im, erosion_dst, element);
    //dilate(erosion_dst, dilation_dst, element );
    Mat h_plane;
    Mat s_plane;
    Mat v_plane;
    Mat dst;
    cvtColor(cv_im,cv_im, CV_BGR2HSV);
    cvtColor(cv_im, im_gray,CV_BGR2GRAY);
    h_plane.create(im_gray.size(),im_gray.type());
    s_plane.create(im_gray.size(),im_gray.type());
    v_plane.create(im_gray.size(),im_gray.type());
    vector<Mat>hsv_planes;
    split(cv_im, hsv_planes);
    h_plane = hsv_planes[0];
    s_plane = hsv_planes[1];
    v_plane = hsv_planes[2];
    for(int j = 0;j < im_gray.rows;j++){
        const uchar* data_h = h_plane.ptr<uchar>(j);
        const uchar* data_s = s_plane.ptr<uchar>(j);
        const uchar* data_v = v_plane.ptr<uchar>(j);
        for(int i=0; i<im_gray.cols*im_gray.channels(); i++){
            //if (26<data_h[i]*2&&data_h[i]*2<34&&data_s[i]>100&&data_v[i]>80){
            if (data_h[i]*2>220&&data_h[i]*2<270&&data_s[i]>100&&data_v[i]>40){
                im_gray.at<uchar>(j,i)=0;




            }

            else{

                im_gray.at<uchar>(j,i)=255;

            }




        }

    }
    erode(im_gray, erosion_dst, element);
    dilate(erosion_dst, dilation_dst, element );

    imshow("binary", dilation_dst);
    cv::waitKey(3);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarachy;



    findContours(dilation_dst,contours,hierarachy,CV_RETR_CCOMP,CV_CHAIN_APPROX_NONE);
    vector<vector<Point> > contours_poly(contours.size());
    vector<Rect> boundRect(contours.size());
    perception_msgs::Rect r;
    Mat imagegg = Mat::zeros(dilation_dst.size(),dilation_dst.type());
    for ( int i=0; i<contours.size(); i++){
        approxPolyDP(Mat(contours[i]),contours_poly[i],3,true);
        boundRect[i] = boundingRect(Mat(contours_poly[i]));






    }
    int a;
    int x;
    int b= 0;
    int k = 0 ;
    int ss=128;
    int san=128;



    int c = boundRect[k].width*boundRect[k].height;
    if (c > 107200){
        k=1;
        c= boundRect[k].width*boundRect[k].height;
    }
    for ( int i=2;i<contours.size(); i++){
      if(boundRect[i].height < 2*boundRect[i].width)
          continue;
      a =boundRect[i].width*boundRect[i].height;
      //cout<< "a" << a << "c" << c << endl;
     if (c<a){
         c = a;
         b = i;

     }





    }
    //cout<< "b" << b << endl;
        r.x = boundRect[b].x;
        r.y = boundRect[b].y;
        r.width = boundRect[b].width;
        r.height = boundRect[b].height;
        //rectangle(im_gray,cv::Point(boundRect[i].x,boundRect[i].y),cv::Point(boundRect[i].x+boundRect[i].width,boundRect[i].y+boundRect[i].height),Scalar(0,255,0),2);
        drawContours( imagegg, contours_poly, b, Scalar(255,0,255), 1, 8, vector<Vec4i>(), 0, Point() );
        rectangle(imagegg,boundRect[b].tl(),boundRect[b].br(),Scalar(255,0,255),2);
        bb_pub_.publish(r);
   cout<<float(boundRect[b].x+boundRect[b].width)/640<<float(boundRect[b].x)/640.0<<endl;








    imshow("binary1", imagegg);
    cv::waitKey(3);






    }

//void imagecallback(const sensor_msgs::ImageConstPtr& image){

//    cv_bridge::CvImagePtr cv_ptr;
//    try{

//    cv_ptr = cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::BGR8);
//        }
//    catch(cv_bridge::Exception& e){
//    ROS_ERROR("CVBRIDGE:{} %s",e.what());
//    return;

//        }
//    cv::Mat cv_im;




//void imagecallback(const sensor_msgs::ImageConstPtr& image){

//    cv_bridge::CvImagePtr cv_ptr;
//    try{

//    cv_ptr = cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::BGR8);
//        }
//    catch(cv_bridge::Exception& e){
//    ROS_ERROR("CVBRIDGE:{} %s",e.what());
//    return;

//        }
//    cv::Mat cv_im;


//void imagecallback(const sensor_msgs::ImageConstPtr& image){

//    cv_bridge::CvImagePtr cv_ptr;
//    try{

//    cv_ptr = cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::BGR8);
//        }
//    catch(cv_bridge::Exception& e){
//    ROS_ERROR("CVBRIDGE:{} %s",e.what());
//    return;

//        }
//    cv::Mat cv_im;




//}

};

int main(int argc, char** argv){
    //Facedetection::imagecallback &imagedeal;
    ros::init(argc,argv,"color_segmentation");
    //ros::NodeHandle n;
    //ros::Subscriber image_sub = n.subscribe("/kinect2/hd/image_color",&Facedetection::imagecallback);
    //ros::Publisher bb_pub = n.advertise<perception_msgs::Rect>("face_detection/bb",1);
    //perception_msgs::Rect r;
    //r = imagedeal.r;
    //bb_pub.publish(r);
    Facedetection ic;
    ros::spin();



    return 0;
}
