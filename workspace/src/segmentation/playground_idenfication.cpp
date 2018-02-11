#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
//#include <perception_msgs/Rect.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <vector>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
using namespace cv;
using namespace std;
void Erosion( int, void* );
void Dilation( int, void* );
const double camera_fx =570.3422241210938;
const double camera_fy =570.3422241210938;
const double camera_cx =319.5;
const double camera_cy =239.5;
const double camera_factor = 1000;
string color_s;
std_msgs::String color__;
bool kg = false;
bool flage = false;
ros::ServiceClient client;
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

    bb_pub_ = nh_.advertise<std_msgs::String>("/playground",1);


    image_sub_ = it_.subscribe("/camera/rgb/image_rect_color",1,&Facedetection::imagecallback,this);

    //image_depth_sub_ = it_.subscribe("/camera/depth/image",1,&Facedetection::imagecallback1,this);
    //namedWindow("Face");

    }
    ~Facedetection()
    {
       // destroyWindow("Face");
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

    if(flage){

    cv_bridge::CvImagePtr cv_ptr;
    try{

    cv_ptr = cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::BGR8);
        }
    catch(cv_bridge::Exception& e){
    ROS_ERROR("CVBRIDGE:{} %s",e.what());
    return;

        }
    cv::Mat cv_im1;
    cv::Mat cv_im;
    GaussianBlur(cv_ptr->image,cv_im1,cv::Size(13,13),0,0);
    Mat kernel = (Mat_<int>(3,3) << 0,-1,0,-1,5,-1,0,-1,0);
    filter2D(cv_im1,cv_im,cv_im1.depth(),kernel);

//    cv_im1 = cv_ptr->image;
//    Mat cv_im(cv_im1.size(),CV_32FC3);
//    cout<<cv_im.size()<<endl;
//    for (int i=0; i<cv_im.rows;i++){

//        for (int j=0; j<cv_im.cols;j++){
//            cv_im.at<Vec3f>(i,j)[0] = pow(cv_im1.at<Vec3b>(i,j)[0],3);
//            cv_im.at<Vec3f>(i,j)[1] = pow(cv_im1.at<Vec3b>(i,j)[1],3);
//            cv_im.at<Vec3f>(i,j)[2] = pow(cv_im1.at<Vec3b>(i,j)[2],3);




//        }



//    }
//    normalize(cv_im,cv_im,0,255,CV_MINMAX);
//    convertScaleAbs(cv_im,cv_im);
    //cv::Mat im_gray;
    //cvtColor(cv_ptr->image,im_gray, CV_BGR2GRAY);
    //imshow("a",cv_im);
    //cv::waitKey(3);
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(20,20));
    Mat erosion_dst,dilation_dst,im_gray, im_gray_blue,erosion_dst1,dilation_dst1;
    //erode(cv_im, erosion_dst, element);
    //dilate(erosion_dst, dilation_dst, element );
    Mat h_plane;
    Mat s_plane;
    Mat v_plane;
    Mat dst;
    cvtColor(cv_im,cv_im, CV_BGR2HSV);
    cvtColor(cv_im, im_gray,CV_BGR2GRAY);
    cvtColor(cv_im, im_gray_blue,CV_BGR2GRAY);
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
            if (55<data_h[i]*2&&data_h[i]*2<62&&data_s[i]>80&&data_v[i]>20){
            //if (data_h[i]*2>220&&data_h[i]*2<270){
            //if (data_h[i]*2>60&&data_h[i]*2<120&&data_s[i]>20&&data_v[i]>10){
                im_gray.at<uchar>(j,i)=0;




            }

            else{

                im_gray.at<uchar>(j,i)=255;

            }




        }

    }
    erode(im_gray, erosion_dst, element);
    dilate(erosion_dst, dilation_dst, element );

    //imshow("binary", dilation_dst);
    //cv::waitKey(3);

    for(int j = 0;j < im_gray_blue.rows;j++){
        const uchar* data_h = h_plane.ptr<uchar>(j);
        const uchar* data_s = s_plane.ptr<uchar>(j);
        const uchar* data_v = v_plane.ptr<uchar>(j);
        int k= 0;
        for(int i=0; i<im_gray_blue.cols*im_gray_blue.channels(); i++){
            if (52<data_h[i]*2&&data_h[i]*2<68&&data_s[i]>100&&data_v[i]>100){
            //if (data_h[i]*2>180&&data_h[i]*2<270&&data_s[i]>40&&data_v[i]>20){
            //if (data_h[i]*2>60&&data_h[i]*2<120&&data_s[i]>20&&data_v[i]>10){
                im_gray_blue.at<uchar>(j,i)=0;




            }

            else{

                im_gray_blue.at<uchar>(j,i)=255;

            }




        }

    }
    erode(im_gray_blue, erosion_dst1, element);
    dilate(erosion_dst1, dilation_dst1, element );


    vector<vector<Point> > contours;
    vector<Vec4i> hierarachy;



    findContours(dilation_dst,contours,hierarachy,CV_RETR_CCOMP,CV_CHAIN_APPROX_NONE);
    vector<vector<Point> > contours_poly(contours.size());
    vector<Rect> boundRect(contours.size());
    //perception_msgs::Rect r;
    Mat imagegg = Mat::zeros(dilation_dst.size(),dilation_dst.type());
    for ( int i=0; i<contours.size(); i++){
        approxPolyDP(Mat(contours[i]),contours_poly[i],3,true);
        boundRect[i] = boundingRect(Mat(contours_poly[i]));






    }
    int a;
    int b= 0;
    int k = 0;
    int g = 0;

    int c = 0;
//    if (c > 107200){
//        k=1;
//        c= boundRect[k].width*boundRect[k].height;
//    }
cout << contours.size() << endl;
    for ( int i=0;i<contours.size(); i++){
//      if( boundRect[i].width < boundRect[i].height )
//        //if(  boundRect[i].height < 2*boundRect[i].width)
//        continue;
      a = boundRect[i].width*boundRect[i].height;
      //

//cout<< "a" << a << "c" << c << endl;

      if(a >40000){
     
      continue;
}
      if (a< 10000)continue;
      //cout << a << endl;
      //cout << i << endl;
      if (c<a){
         c = a;
         b = i;
         g++;

     }





    }
    //cout<< "b" << b << endl;
        /*//r.x = boundRect[b].x;
        r.y = boundRect[b].y;
        r.width = boundRect[b].width;
        r.height = boundRect[b].height;*/
cout << boundRect[b].width * boundRect[b].height << endl;
        //rectangle(im_gray,cv::Point(boundRect[i].x,boundRect[i].y),cv::Point(boundRect[i].x+boundRect[i].width,boundRect[i].y+boundRect[i].height),Scalar(0,255,0),2);
        drawContours( imagegg, contours_poly, b, Scalar(255,255,0), 1, 8, vector<Vec4i>(), 0, Point() );
        rectangle(imagegg,boundRect[b].tl(),boundRect[b].br(),Scalar(255,255,0),2);
 //   std_msgs::String color;
   k= boundRect[b].width * boundRect[b].height;
   std_srvs::SetBool srv;
   if (k>40000) cout<< "nothing" <<endl;

    else if (g>0){

       srv.request.data = true;
   cout<< "yellow" << endl;
    }
        else {

            srv.request.data = false;
    cout<< "blue" << endl;

        }

    if(client.call(srv)){

	flage = false;
    //ros::spinOnce();
    }
    else{
	
	flage = true;
        ROS_ERROR("Failed to call service color_segmentation");


    }

    //bb_pub_.publish(color_s);
  // cout<<boundRect[b].width << boundRect[b].height <<endl;
//    for ( int i=0; i<contours.size(); i++){

//        r.x = boundRect[i].x;
//        r.y = boundRect[i].y;
//        r.width = boundRect[i].width;
//        r.height = boundRect[i].height;
//        //rectangle(im_gray,cv::Point(boundRect[i].x,boundRect[i].y),cv::Point(boundRect[i].x+boundRect[i].width,boundRect[i].y+boundRect[i].height),Scalar(0,255,0),2);
//        drawContours( imagegg, contours_poly, i, Scalar(255,0,255), 1, 8, vector<Vec4i>(), 0, Point() );
//        rectangle(imagegg,boundRect[i].tl(),boundRect[i].br(),Scalar(255,0,255),2);
//        bb_pub_.publish(r);





//    }









    //imshow("binary1", imagegg);
    //cv::waitKey(3);






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





//}
}
};

//    bool farbe(std_srvs::SetBool::Request &req,std_srvs::SetBool::Response &res){
//    res.message=color_s;
//    kg = !(req.data);
//    //  ROS_INFO(“sending back response:”);
//    cout<< res.message << endl;
//    return true;
//    }

    /*bool to_main(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res){

        

        //kg = true;
        Facedetection ic;
        ros::Rate r(0.1);
        r.sleep();
        //ros::spinOnce();
        

        
    }*/
    bool to_main(std_srvs::Empty::Request &req1,std_srvs::Empty::Response &res1){

        

        //kg = true;
       	
       //destroyAllWindows();
       return true;
        

        
    }

    bool start(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res){



        //kg = true;
       flage = true;

    return true;

    }





int main(int argc, char** argv){
    //Facedetection::imagecallback &imagedeal;
    ros::init(argc, argv, "playground_identification");
    ros::NodeHandle n;
    /*

    geometry_msgs::Twist msg2;
   for(int i=0;i<6;i++){
    msg2.linear.x=0.45;
    msg2.angular.z=0;
    cout<<1 << endl;
    //ros::Duration(1);
    vel_pub.publish(msg2);
    rate.sleep();
    ros::spinOnce();
}
    rate.sleep();
   for(int i=0;i<8;i++){
    msg2.linear.x=0;
    msg2.angular.z=1;
    cout<<1 << endl;
    //ros::Duration(1);
    vel_pub.publish(msg2);
    rate.sleep();
    ros::spinOnce();
}*/
     

    //ros::ServiceServer service = n.advertiseService("color_detection",farbe);

    ROS_INFO("Ready to detecte color.");
    client = n.serviceClient<std_srvs::SetBool>("get_the_color");
    ros::ServiceServer service1 = n.advertiseService("playground",to_main);
    ros::ServiceServer service2 = n.advertiseService("start",start);
    /*if(kg){
    Facedetection ic;
}*/
    Facedetection ic;
    ros::spin();



    return 0;
}
