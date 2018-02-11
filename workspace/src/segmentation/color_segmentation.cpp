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
string color="blue";
bool kg = false;

bool s=false;

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

    bb_pub_ = nh_.advertise<geometry_msgs::Point>("/color_segmentation/bb",1);


    image_sub_ = it_.subscribe("/camera/rgb/image_rect_color",1,&Facedetection::imagecallback,this);

    //image_depth_sub_ = it_.subscribe("/camera/depth_registered/image",1,&Facedetection::imagecallback1,this);
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
    cout<< "1" << endl;

    if(color=="blue"||color=="yellow"){

    cout<< "2" << endl;


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
        for(int j=0; j<640;j++){
            cha[0].at<uchar>(i,j)=0;
            cha[1].at<uchar>(i,j)=0;
            cha[2].at<uchar>(i,j)=0;


        }



    }
    merge(cha,result);








    GaussianBlur(result,cv_im1,cv::Size(13,13),0,0);
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
    imshow("a",cv_im);
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
//    for(int j = 0;j < im_gray.rows;j++){
//        const uchar* data_h = h_plane.ptr<uchar>(j);
//        const uchar* data_s = s_plane.ptr<uchar>(j);
//        const uchar* data_v = v_plane.ptr<uchar>(j);
//        for(int i=0; i<im_gray.cols*im_gray.channels(); i++){
//            if (230<data_h[i]*2&&data_h[i]*2<260&&data_s[i]>70&&data_v[i]>60){
//            //if (data_h[i]*2>220&&data_h[i]*2<270){
//            //if (data_h[i]*2>60&&data_h[i]*2<120&&data_s[i]>20&&data_v[i]>10){
//                im_gray.at<uchar>(j,i)=0;



//    Mat h_sandra;
//    Mat s_d;
//    Mat v_s;






//            }

//            else{

//                im_gray.at<uchar>(j,i)=255;

//            }




//        }

//    }


    //std_msgs::String color;


    //=========================================================================

        for(int j = 0;j < im_gray.rows;j++){
            const uchar* data_h = h_plane.ptr<uchar>(j);
            const uchar* data_s = s_plane.ptr<uchar>(j);
            const uchar* data_v = v_plane.ptr<uchar>(j);

            for(int i=0; i<im_gray.cols*im_gray.channels(); i++){


//                cout<< "3" << endl;

            if(color=="blue"){



                   if (230<data_h[i]*2&&data_h[i]*2<270  &&  data_s[i]>60  &&  data_v[i]>50){
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

                       if (52<data_h[i]*2<70  &&  data_s[i]>130  &&  data_v[i]>120){

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
     cout<< "6666666" << endl;
//=============================================================================0


    erode(im_gray, erosion_dst, element);
    dilate(erosion_dst, dilation_dst, element );

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
    double max = 500;
    int index = 0;
    for ( int i=0; i<contours.size(); i++)
    {
        approxPolyDP(Mat(contours[i]),contours_poly[i],3,true);
        boundRect[i] = boundingRect(Mat(contours_poly[i]));
        double size = boundRect[i].width*boundRect[i].height;
        if((size > max)&&(boundRect[i].height>2*boundRect[i].width)&&(boundRect[i].height<5*boundRect[i].width))
        {
            max = size;
            index = i;
            hockeyNum ++;
        }




                }
    if (hockeyNum ==0) cout << "nothing" << endl;
        geometry_msgs::Point p;
                if(hockeyNum != 0){
                    r.x = boundRect[index].x;
                    r.y = boundRect[index].y;
                    r.width = boundRect[index].width;
                    r.height = boundRect[index].height;
                     bb_pub_.publish(p);
                      p.x = boundRect[index].x + boundRect[index].width/2;
                      p.y = boundRect[index].y + boundRect[index].height/2;
                    drawContours( imagegg, contours_poly, index, Scalar(255,0,255), 1, 8, vector<Vec4i>(), 0, Point() );
                    rectangle(imagegg,r.tl(),r.br(),Scalar(255,0,255),2);




                imshow("binary1", imagegg);
                cv::waitKey(3);

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






//         }

//     //void imagecallback(const sensor_msgs::ImageConstPtr& image){

//     //    cv_bridge::CvImagePtr cv_ptr;
//     //    try{

//     //    cv_ptr = cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::BGR8);
//     //        }
//     //    catch(cv_bridge::Exception& e){
//     //    ROS_ERROR("CVBRIDGE:{} %s",e.what());
//     //    return;

//     //        }
//     //    cv::Mat cv_im;



//     }



//     cout<< "11111111" << endl;





//}


};

//bool to_main(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res){

//        //while (ros::ok())
//        kg = true;
//        Facedetection ic;
//    }






    bool main_(std_srvs::SetBool::Request &req1,std_srvs::SetBool::Response &res1){
        s=req1.data;
        if(s==true){
            color="yellow";
        }
        else{
            color="blue";
        }
        res1.message=color;

    cout<< "4" << endl;

//        res1.message=color_s;
//    kg = !(req1.data);
//  ROS_INFO(“sending back response:”);
    cout<< color << endl;
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

//    std_srvs::SetBool srv;
//    srv.request.data = true;
//    cout << kg << endl;
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
    cout<< "5" << endl;

    Facedetection ic;
    ros::spin();



    return 0;
}

































