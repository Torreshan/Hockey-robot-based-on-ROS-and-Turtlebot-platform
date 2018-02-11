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
#include <algorithm>
#include <string>
#include <set>
#include <iterator>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
using namespace cv;
using namespace std;
void Erosion( int, void* );
void Dilation( int, void* );
const double camera_fx =570.3422241210938;
const double camera_fy =570.3422241210938;
const double camera_cx =319.5;
const double camera_cy =239.5;
const double camera_factor = 1000;
string color;


class Facedetection{


    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    //ros::Publisher bb_pub_;
    ros::Publisher pointss;
    image_transport::Subscriber image_sub_;
    //image_transport::Subscriber image_depth_sub_;
public:
    Facedetection()
        : it_(nh_)
    {

    //bb_pub_ = nh_.advertise<perception_msgs::Rect>("/color_segmentation/bb",1);

    pointss = nh_.advertise<geometry_msgs::PoseArray>("/line/points",1);
    image_sub_ = it_.subscribe("/camera/rgb/image_rect_color",1,&Facedetection::imagecallback,this);

    //image_depth_sub_ = it_.subscribe("/camera/depth/image",1,&Facedetection::imagecallback1,this);
    namedWindow("Face");

    }
    ~Facedetection()
    {
        destroyWindow("Face");
    }
    /*Mat cv_im;
    Mat im_gray;
    Mat im_x;
    Mat im_y;
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
    cv::Mat cv_im1;
    cv::Mat cv_im;
    cv::Mat cv_1;
    cv::Mat cv_2;
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
    for(int j = 0;j < im_gray.rows;j++){
        const uchar* data_h = h_plane.ptr<uchar>(j);
        const uchar* data_s = s_plane.ptr<uchar>(j);
        const uchar* data_v = v_plane.ptr<uchar>(j);
        for(int i=0; i<im_gray.cols*im_gray.channels(); i++){
            //if (55<data_h[i]*2&&data_h[i]*2<62&&data_s[i]>40&&data_v[i]>20){
            //if (data_h[i]*2>220&&data_h[i]*2<270){
            if (data_h[i]*2>70&&data_h[i]*2<154&&data_s[i]>43&&data_v[i]>46){
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
    //perception_msgs::Rect r;
    Mat imagegg = Mat::zeros(dilation_dst.size(),dilation_dst.type());
    
    for ( int i=0; i<contours.size(); i++){
        approxPolyDP(Mat(contours[i]),contours_poly[i],3,true);
        boundRect[i] = boundingRect(Mat(contours_poly[i]));


    



    }
    
   
    int a;
    int b= 0;
    int k = 100 ;
    vector<cv::Rect> ind;
    vector<Point> mid;
    int c = 0;
   // int h = 0;
    
    for ( int i=0;i<contours.size(); i++){
      //if( 4*boundRect[i].width > boundRect[i].height > 2*boundRect[i].width)

    if(  boundRect[i].height < 6*boundRect[i].width)
        continue;
      a =boundRect[i].width*boundRect[i].height;
      //cout<< "a" << a << "c" << c << endl;
      if (a < 500)continue;
     if (c<a){
         c = a;
         b = i;




         if(ind.size()>0){
         for(int j = 0; j< ind.size();j++){

                if((ind[j].x-25)<boundRect[i].x<(ind[j].x+25)){
                    continue;
                    k=j;
                }



         }
         }
         //cout << ind.size() << endl;
         //cout << k << endl;
         if((k+1) < ind.size()){
             continue;
         }
//         cout << "aaaa"<<ind.size() << endl;
         ind.push_back(boundRect[i]);

//         cout << ind.size() << endl;
 


        drawContours( imagegg, contours_poly, b, Scalar(255,0,255), 1, 8, vector<Vec4i>(), 0, Point() );
        rectangle(imagegg,boundRect[b].tl(),boundRect[b].br(),Scalar(255,0,255),2);
    
     }
}

     //double wid = abs(boundRect[b].br().x - boundRect[b].tl().x);
     //double hei = abs(boundRect[b].br().y - boundRect[b].tl().y);
     if(ind.size()==3){
     for(int i =0;i<ind.size();i++){

     if(ind[i].br().x < 640 && ind[i].br().y < 480 ){
     cv::Point ii;
     double wid = ind[i].width;
     double hei = ind[i].height;
     ii.x= ind[i].x + wid/2;
     ii.y = ind[i].y + hei/2;
//     cout<<ii.x<<endl;
//     cout<<ii.y<<endl;
     mid.push_back(ii);
      //mid[i].x =ind[i].tl().x + wid/2;
      //mid[i].y = ind[i].tl().y + hei/2;
      //h++;
    
     }
     }
     }


  //******************************************************************************************************************//
  // get the series of all of the detected Pfosten
  //  cout << mid.size() <<endl;
    if (mid.size() ==3){
//        cout<<"MID0 IS"<<mid[0]<<endl;
//        cout<<"MID1 IS"<<mid[1]<<endl;
//        cout<<"MID2 IS"<<mid[2]<<endl;

        //vector<double> dist;
        vector<double> temp;
        //for(int i = 1;i<=3;i++)
        //    for(int j = )
        struct distanceBetween
        {
            std::vector<int> subscript; //柱子
            double value;
        }dist[3];
       
        dist[0].value = sqrt(pow(mid[0].x - mid[1].x,2)+pow(mid[0].y - mid[1].y,2));
        //dist[0].subscript = {1,2};//1,2
        
        dist[0].subscript.push_back(0);
        dist[0].subscript.push_back(1);
        dist[1].value = sqrt(pow(mid[0].x - mid[2].x,2)+pow(mid[0].y - mid[2].y,2));
        
        //dist[1].subscript = {1,3};//1,3
        dist[1].subscript.push_back(0);
        dist[1].subscript.push_back(2);
        dist[2].value = sqrt(pow(mid[1].x - mid[2].x,2)+pow(mid[1].y - mid[2].y,2));
        
        //dist[2].subscript = {2,3};//2,3
        dist[2].subscript.push_back(1);
        dist[2].subscript.push_back(2);
        
        //set<double>ddist = {dist[0].value,dist[1].value,dist[2].value};
       
//      cout<<dist[0].subscript[0]<<endl;
//      cout<<dist[0].subscript[1]<<endl;

//        cout<<dist[1].subscript[0]<<endl;
//      cout<<dist[1].subscript[1]<<endl;

//        cout<<dist[2].subscript[0]<<endl;
//       cout<<dist[2].subscript[1]<<endl;
        cout<<dist[0].value<<endl;
        cout<<dist[1].value<<endl;
        cout<<dist[2].value<<endl;

        std::vector<double> ddist;
        ddist.push_back(dist[0].value);
        ddist.push_back(dist[1].value);
        ddist.push_back(dist[2].value);


        sort(ddist.begin(), ddist.end());
//        double ddist[3]={dist[0].value,dist[1].value,dist[2].value};
//        cout<<ddist[0]<<endl;
//        cout<<ddist[1]<<endl;
//        cout<<ddist[2]<<endl;
//        //sort (ddist,ddist+3);


        //vector<double>::iterator biggest = max_element(begin(dist), end(dist));
        //vector<double>::iterator smallest = min_element(begin(dist), end(dist));
        std::vector<double>::iterator biggest = max_element(ddist.begin(), ddist.end());
        std::vector<double>::iterator smallest = min_element(ddist.begin(), ddist.end());

        int big_index = distance(ddist.begin(), biggest);
        int min_index = distance(ddist.begin(), smallest);
//        cout<<"the index of biggest is "<<big_index<<endl;
//        cout<<"the index of minimum is "<<min_index<<endl;


//        temp.push_back(*biggest);
//        temp.push_back(*smallest);
//        sort(temp.begin(), temp.end());
//        //cout<<"temp is"<<temp[0]<<endl;
//        //cout<<"temp is"<<temp[1]<<endl;
//        //double temp[2] = {*biggest,~smallest};
//        //sort(temp,temp+2);

//        std::vector<double> midd(20);
        
     
//        std::vector<double>::iterator it = set_difference(ddist.begin(),ddist.end(),temp.begin(),temp.end(),midd.begin());

//        midd.resize(it-midd.begin());
//        int mid_index = distance(ddist.begin(),midd.begin());

         // get the middle_index
        int mid_index = 0;
        for(int i=0;i<3;i++){
            if(i!=big_index and i!=min_index)
                mid_index = i;
        }

      //  cout<<big_index<<endl;
      //  cout<<mid_index<<endl;
      //  cout<<min_index<<endl;



        double bigs[] = {dist[big_index].subscript[0],dist[big_index].subscript[1]};
        double mids[] = {dist[mid_index].subscript[0],dist[mid_index].subscript[1]};
        double mins[] = {dist[min_index].subscript[0],dist[min_index].subscript[1]};


        std::sort (bigs,bigs+2);
        std::sort (mids,mids+2);
        std::sort (mins,mins+2);

        std::vector<int> v1(10);
        std::vector<int> v2(10);
        std::vector<int> v3(10);

        std::vector<int>::iterator it1=std::set_intersection (bigs, bigs+2, mids, mids+2, v1.begin());
        std::vector<int>::iterator it2=std::set_intersection (bigs, bigs+2, mins, mins+2, v2.begin());
        std::vector<int>::iterator it3=std::set_intersection (mids, mids+2, mins, mins+2, v3.begin());


//        std::vector<int> v1(20);
//        std::vector<int> v2(20);
//        std::vector<int> v3(20);
//        std::sort(v1.begin(), v1.end());
//        std::sort(v2.begin(), v2.end());
//        std::sort(v3.begin(), v3.end());


//        std::vector<int>::iterator it1 = set_intersection(dist[big_index].subscript.begin(),dist[big_index].subscript.end(),dist[mid_index].subscript.begin(),dist[mid_index].subscript.end(),v1.begin());
//        std::vector<int>::iterator it2 = set_intersection(dist[big_index].subscript.begin(),dist[big_index].subscript.end(),dist[min_index].subscript.begin(),dist[min_index].subscript.end(),v2.begin());
//        std::vector<int>::iterator it3 = set_intersection(dist[mid_index].subscript.begin(),dist[mid_index].subscript.end(),dist[min_index].subscript.begin(),dist[min_index].subscript.end(),v3.begin());

        v1.resize(it1-v1.begin());
        v2.resize(it2-v2.begin());
        v3.resize(it3-v3.begin());
       // cout<<11111111111111111<<endl;

        int left = v1[0];
        int right = v2[0];
        int middle = v3[0];
        // cout<<left<<endl;
        // cout<<right<<endl;
        // cout<<middle<<endl;
        //geometry_msgs::PoseStamped pose;
        geometry_msgs::Pose pose;
        geometry_msgs::PoseArray PoseArray;
       //  cout<<"final"<<endl;
        pose.position.x = mid[left].x;
        pose.position.y = mid[left].y;

       // PoseArray.poses[0] = pose;
        PoseArray.poses.push_back(pose);
        pose.position.x = mid[middle].x;
        pose.position.y = mid[middle].y;
        // PoseArray.poses[1] = pose;
        PoseArray.poses.push_back(pose);



        pose.position.x = mid[right].x;
        pose.position.y = mid[right].y;
        //PoseArray.poses[2] = pose;
        PoseArray.poses.push_back(pose);
         cout<<"final"<<endl;

        pointss.publish(PoseArray);





 //**********************************************************************************************************//


    }













//int maxdata = 0;

       //std::vector<int>::iterator biggest = std::max_element(std::begin(ind),std::end(ind));
       //std::vector<int>::iterator smallest = std::small_element(std::begin(ind),std::end(ind));
  //     for(int i=0;i<ind.size();i++){
  //     if(ind[i]>maxdata)

    //       maxdata = ind[i];


     //  }
     //cout << maxdata  << endl;




    //cout<< "b" << b << endl;
       // r.x = boundRect[b].x;
       // r.y = boundRect[b].y;
       // r.width = boundRect[b].width;
       // r.height = boundRect[b].height;
        //rectangle(im_gray,cv::Point(boundRect[i].x,boundRect[i].y),cv::Point(boundRect[i].x+boundRect[i].width,boundRect[i].y+boundRect[i].height),Scalar(0,255,0),2);
       // drawContours( imagegg, contours_poly, b, Scalar(255,0,255), 1, 8, vector<Vec4i>(), 0, Point() );
        //rectangle(imagegg,boundRect[b].tl(),boundRect[b].br(),Scalar(255,0,255),2);
        //bb_pub_.publish(r);
   //cout<<boundRect[b].width<<endl;
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





//}

};


int main(int argc, char** argv){
    //Facedetection::imagecallback &imagedeal;
    ros::init(argc,argv,"line");
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
