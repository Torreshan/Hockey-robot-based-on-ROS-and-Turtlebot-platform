#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtle_vis/myClass/TurtleClass.h>
#include "collision_prevention.h"


/** The robot_driver node drives the robot to the target location requested by the navigation node.
This process is realized by combining TurtleClass, the server responsible for calculating the linear and
angular speed to the target, with collision_prevention, the class that uses lidar scan messages to
avoid obstacles along the way and plan the path.  */

int main(int argc, char	**argv)	{
    ros::init(argc, argv, "robot_driver");

    ros::NodeHandle nh;

    //TurtleClass: server responsible for navigation, i.e. it calculates velocity for the requested location.
    turtleSpace::TurtleClass turtleF;

    /** Collision_Prevention: responsible for obstacle avoidance, i.e. for a given input velocity, col_prev
    evaluates turtle's surroundings and makes the decision whether the turtle should execute the input
    speed command or dodge a detected obstacle. */
    collision_prevention col_prevention;

    // Instantialize a publisher for publishing velocity messages
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);

    // Instantialize a laser scan subscriber for col_prevention to detect obstacles
    ros::Subscriber laser_sub = nh.subscribe("/lidarscan",
                                             5,
                                             &collision_prevention::scanCallback,
                                             &col_prevention);

    // Service for driving tasks
    ros::ServiceServer drive_srv = nh.advertiseService("call_driver",
                                                       &collision_prevention::driveSrvCallback,
                                                       &col_prevention);

    // Advertise a service for GOTO-target-location requests
    ros::ServiceServer service = nh.advertiseService("TurtlePose",
                                                     &turtleSpace::TurtleClass::getDPose,
                                                     &turtleF);

    // Subscribe to /odom in order to calculate velocity
    ros::Subscriber sub = nh.subscribe("/odom",100,
                                          &turtleSpace::TurtleClass::getTiagoPose,
                                          &turtleF);

    ros::Rate rate(100);

    //turtleTIAGOPose_desired_local_2d << 1,1;

    Vector2d error_2d;
    Vector2d turtleVel_2d;


        //move forward
        ROS_INFO("move forward");
        ros::Time start = ros::Time::now();
        while(ros::ok() && current_pose.x < 1.5) {
            geometry_msgs::Twist move; //velocity controls
            move.linear.x = 0.2; //speed value m/s
            move.angular.z = 0;
            movement_pub.publish(move);
            ros::spinOnce();
            rate.sleep();
        }
        //turn right
        ROS_INFO("turn right");
        ros::Time start_turn = ros::Time::now();
        while(ros::ok() && current_pose.theta > -PI/2)
        {
            geometry_msgs::Twist move;
            //velocity controls
            move.linear.x = 0; //speed value m/s
            move.angular.z = -0.3;
            movement_pub.publish(move);

            ros::spinOnce();
            rate.sleep();

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




                    }
            }

                erode(im_gray, erosion_dst, element);
                dilate(erosion_dst, dilation_dst, element );

                imshow("binary", dilation_dst);
                cv::waitKey(3);
                vector<vector<Point> > contours;
                vector<Vec4i> hierarachy;




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
              // get the series of all of the detected  柱子
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
                    using namespace cv;
                    using namespace std;

                    int forward = 5;
                    double speed = 0.3;
                    double searchspeed = 0.2;
                    int hockeyNum = 0;
                    bool isForward = false;
                    //static int hockeyNum = 0;
                    bool flag = true;
                    bool catch_state = false;
                    geometry_msgs::Twist msg;

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
                        void Turn(double turnAngle)
                        {

                            msg.linear.x = 0;
                            msg.angular.z = turnAngle;
                    //        if(( msg.linear.x!=0)|( msg.angular.z!=0))
                    //        aim_pub.publish(msg)
                    ;
                        }
                        void antiTurn(double turnAngle)
                        {

                            msg.linear.x = 0;
                            msg.angular.z = -turnAngle;
                    //        if(( msg.linear.x!=0)|(  int hockeyNum = 0;msg.angular.z!=0))
                    //        aim_pub.publish(msg);
                        }
                        void Moveforward()
                        {

                            msg.linear.x = 0.4;
                            msg.angular.z = 0.000000001;
                    //        if(( msg.linear.x!=0)|( msg.angular.z!=0))
                    //        aim_pub.publish(msg);int hockeyNum = 0;
                        }
                        bool callback(std_srvs::Empty::Request& e,std_srvs::Empty::Response& ee)
                        {
                            //flag = true;
                            return flag + 1;
                        }

                        void imagecallback(const sensor_msgs::ImageConstPtr& image)
                        {
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

                         //   tf::TransformListener listener_;

                            merge(cha,result);
                            GaussianBlur(result,cv_im1,cv::Size(13,13),0,0);
                            Mat kernel = (Mat_<int>(3,3) << 0,-1,0,-1,5,-1,0,-1,0);
                            filter2D(cv_im1,cv_im,cv_im1.depth(),kernel);
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
                            //int rows = im_gray.rows;
                            int columns = im_gray.cols;
                            int rows = im_gray.rows;
                            int area = columns * rows;
                            int keypoint = 0;


                            for(int j = 0;j < rows;j++)
                            {
                                const uchar* data_h = h_plane.ptr<uchar>(j);
                                const uchar* data_s = s_plane.ptr<uchar>(j);
                                const uchar* data_v = v_plane.ptr<uchar>(j);
                                for(int i=0; i<columns*im_gray.channels(); i++)
                                {

                                   // if (55<data_h[i]*2&&data_h[i]*2<62&&data_s[i]>40&&data_v[i]>20)
                                   // {
                                    if(data_h[i]*2>220&&data_h[i]*2<270&&data_s[i]>60&&data_v[i]>40){
                                    //if (data_h[i]*2>60&&int hockeyNum = 0;data_h[i]*2<120&&data_s[i]>20&&data_v[i]>10){
                                        im_gray.at<uchar>(j,i)=0;
                                        keypoint ++;
                                        if(keypoint / area >=0.25)
                                        {
                                            catch_state = true;
                                            hockeyNum=0;
                                        }


                                    }

                                    else
                                    {

                                        im_gray.at<uchar>(j,i)=255;
                                        //background_num ++;

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
                            //vector<Rect> rightRect;
                            //Rect rightBound;
                            //perception_msgs::Rect r;
                            cv::Rect r;

                            Mat imagegg = Mat::zeros(dilation_dst.size(),dilation_dst.type());
                            double max = 3000;
                            int index = 0;


                            for ( int i=0; i<contours.size(); i++)
                            {
                                approxPolyDP(Mat(contours[i]),contours_poly[i],3,true);
                                boundRect[i] = boundingRect(Mat(contours_poly[i]));
                                double size = boundRect[i].width*boundRect[i].height;
                                if((size > max)&&(boundRect[i].height>2.5*boundRect[i].width))
                                {
                                    max = size;
                                    index = i;
                                    hockeyNum ++;
                                }




                                        }

                                        if(hockeyNum == 0)
                                        {
                                            forward = 2;
                                            //double try = 0.1;
                                            Turn(searchspeed);
                                            //ros::Duration().sleep();
                                            ROS_INFO_STREAM("=========Turn to seach ball===========");

                                        }

                                       else {
                                            r.x = boundRect[index].x;
                                            r.y = boundRect[index].y;
                                            r.width = boundRect[index].width;
                                            r.height = boundRect[index].height;





                                //        int a;
                                //        int b= 0;
                                //        int k = 0 ;

                                //        int c = boundRect[k].width*boundRect[k].height;

                                //        if (c > 107200)
                                //        {
                                //            k=1;
                                //            c= boundRect[k].width*boundRect[k].height;
                                //        }
                                //        for (int i=2;i<contours.size(); i++)
                                //        {
                                //          //if( 4*boundRect[i].width > boundRect[i].height > 2*boundRect[i].width)
                                //            if(  boundRect[i].height < 2*boundRect[i].width)
                                //            continue;
                                //          a =boundRect[i].width*boundRect[i].height;
                                //          //cout<< "a" << a << "c" << c << endl;
                                //             if (c<a)
                                //             {
                                //             c = a;
                                //             b = i;
                                //             }
                                //        }
                                        //cout<< "b" << b << endl;



                                            //rectangle(im_gray,cv::Point(boundRect[i].x,boundRect[i].y),cv::Point(boundRect[i].x+boundRect[i].width,boundRect[i].y+boundRect[i].height),Scalar(0,255,0),2);
                                            drawContours( imagegg, contours_poly, index, Scalar(255,0,255), 1, 8, vector<Vec4i>(), 0, Point() );
                                            rectangle(imagegg,r.tl(),r.br(),Scalar(255,0,255),2);
                                           // bb_pub_.publish(r);










                                        imshow("binary1", imagegg);
                                        cv::waitKey(3);


                                //         forward = 1;
                                //         Moveforward();
                                        // double dt = (current_time - last_time).toSec();
                                        //double antispeed = -0.2;


                                        if(abs(r.tl().x + 0.5*r.width-0.5*columns)>20 && !isForward)
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


                                            //ROS_INFO_STREAM("==========Turn ==========");
                                        }
                                        else

                                        {
                                            forward = 1;
                                            isForward = true;
                                          //  Moveforward();
                                            ROS_INFO_STREAM("=========Moveforward===========");
                                        }
                                       }




                                       }

                                //    void lasercallback(const sensor_msgs::LaserScanConstPtr &scan)
                                //    {
                                //        double angle_min = scan->angle_min;
                                //        double angle_increment = scan->angle_increment;

                                //        double prev_range = scan->ranges[0];
                                //        double curr_range = 0;
                                //        double angle_range = 0; //curr angle of min/maxima of same ranges


                                //    }




                                    int main(int argc, char** argv)
                                    {
                                       // bool run = false;
                                        ros::init(argc,argv,"color_segmentation");



                                        ros::NodeHandle nh_;
                                        image_transport::ImageTransport it_(nh_);
                                        ros::Publisher aim_pub;
                                        ros::ServiceServer state_pub;
                                       // ros::Subscriber laser_sub;
                                        image_transport::Subscriber image_sub_;
                                        namedWindow("hockey");

                                        aim_pub = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
                                        state_pub = nh_.advertiseService("catching_state",callback);
                                        image_sub_ = it_.subscribe("/camera/rgb/image_raw",1,imagecallback);
                                       // laser_sub = nh_.advertise("/lidarscan",lasercallback);
                                        ROS_INFO_STREAM("========== start ==========");
                                //        ros::Rate(0.1);
                                              while(ros::ok())
                                             {
                                                  if(flag == false)
                                                      break;
                                                  if(catch_state == true)
                                                     isForward = false;
                                                  if(forward == 1){
                                                      //run = true;
                                                      msg.linear.x = 0.2;
                                                      msg.angular.z = 0.0000000001;
                                                     // image_sub = Null;
                                                  }

                                                  aim_pub.publish(msg);
                                                  ros::spinOnce();


                                             }



                                       destroyWindow("hockey");



                                        return 0;
                                    }

                                    int main(int argc, char** argv){
                                      ros::init(argc, argv, "simple_navigation_goals");
                                    //  ros::NodeHandle n;
                                    //  ros::ServiceClient client1 = n.serviceClient<std_srvs::Empty>("/global_localization");
                                    //  std_srvs::Empty   srv1;
                                    //  ros::Rate loop_rate(10);
                                    //  ros::Duration(3).sleep();
                                    //  if(client1.call(srv1)){
                                    //    ROS_INFO("global localiztion");
                                    //  }
                                    //  else{
                                    //      ROS_ERROR("Failed");

                                    //  }
                                    //  ros::Duration(3).sleep();
                                    //  ros::Publisher pub=n.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel",100);
                                    //  geometry_msgs::Twist msg;
                                    //  for(int i = 0;i<250;i++){
                                    //    msg.angular.z = 0.3;
                                    //    pub.publish(msg);
                                    //    ros::spinOnce();
                                    //    loop_rate.sleep();
                                    //}

                                    //  ros::ServiceClient client2 = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
                                    //  std_srvs::Empty srv2;
                                    //  if(client2.call(srv2)){
                                    //    ROS_INFO("clear");
                                    //  }
                                    //  else{
                                    //      ROS_ERROR("Failed");

                                    //  }

                                    //  //tell the action client that we want to spin a thread by default
                                      MoveBaseClient ac("move_base", true);

                                    //  //wait for the action server to come up
                                      while(!ac.waitForServer(ros::Duration(5.0))){
                                        ROS_INFO("Waiting for the move_base action server to come up");
                                      }

                                      move_base_msgs::MoveBaseGoal goal;

                                      //we'll send a goal to the robot to move 1 meter forward
                                      goal.target_pose.header.frame_id = "base_link";
                                      goal.target_pose.header.stamp = ros::Time::now();

                                      goal.target_pose.pose.position.x = 1;
                                      goal.target_pose.pose.position.y = 0;

                                      goal.target_pose.pose.orientation.z = 0;
                                      goal.target_pose.pose.orientation.w = 1;
                                      ROS_INFO("Sending goal");
                                      ac.sendGoal(goal);

                                      ac.waitForResult();

                                      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                                        ROS_INFO("Hooray, the base moved A");
                                      else
                                        ROS_INFO("The base failed to move forward 1 meter for some reason");

                                    //  goal.target_pose.pose.position.x = -4.01;
                                    //  goal.target_pose.pose.position.y = -6.39;
                                    //  goal.target_pose.pose.orientation.z = 0.91;
                                    //  goal.target_pose.pose.orientation.w = 0.43;
                                    //  ROS_INFO("Sending goal");
                                    //  ac.sendGoal(goal);

                                    //  ac.waitForResult();

                                    //  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                                    //    ROS_INFO("Hooray, the base moved b");
                                    //  else
                                    //    ROS_INFO("The base failed to move forward 1 meter for some reason");


                                    //    goal.target_pose.pose.position.x = -0.88;
                                    //    goal.target_pose.pose.position.y = -12;
                                    //    goal.target_pose.pose.orientation.z = 0.91;
                                    //    goal.target_pose.pose.orientation.w = 0.43;
                                    //    ROS_INFO("Sending goal");
                                    //    ac.sendGoal(goal);

                                    //    ac.waitForResult();

                                    //    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                                    //      ROS_INFO("Hooray, the base moved C");
                                    //    else
                                    //      ROS_INFO("The base failed to move forward 1 meter for some reason");

                                    //    goal.target_pose.pose.position.x = -2.52;
                                    //    goal.target_pose.pose.position.y = -2.24;
                                    //    goal.target_pose.pose.orientation.z = 0.91;
                                    //    goal.target_pose.pose.orientation.w = 0.43;
                                    //    ROS_INFO("Sending goal");
                                    //    ac.sendGoal(goal);

                                    //    ac.waitForResult();

                                    //    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                                    //      ROS_INFO("Hooray, the base moved C");
                                    //    else
                                    //      ROS_INFO("The base failed to move forward 1 meter for some reason");

                                        return 0;
                                      }

                                    const double PI = 3.14159265358979323846;

                                    ROS_INFO("start");

                                    ros::init(argc, argv, "move_pub");
                                    ros::NodeHandle n;
                                    ros::Publisher pub_pose2d = n.advertise("turtlebot_pose2d", 1);
                                    ros::Subscriber sub_odometry = n.subscribe("odom", 1, odomCallback);
                                    ros::Publisher movement_pub = n.advertise("cmd_vel",1); //for sensors the value after , should be higher to get a more accurate result (queued)

                                    ros::Rate rate(10); //the larger the value, the "smoother" , try value of 1 to see "jerk" movement

                                    //move forward
                                    ROS_INFO("move forward");
                                    ros::Time start = ros::Time::now();
                                    while(ros::ok() && current_pose.x < 1.5) {
                                        geometry_msgs::Twist move; //velocity controls
                                        move.linear.x = 0.2; //speed value m/s
                                        move.angular.z = 0;
                                        movement_pub.publish(move);
                                        ros::spinOnce();
                                        rate.sleep();
                                    }
                                    //turn right
                                    ROS_INFO("turn right");
                                    ros::Time start_turn = ros::Time::now();
                                    while(ros::ok() && current_pose.theta > -PI/2)
                                    {
                                        geometry_msgs::Twist move;
                                        //velocity controls
                                        move.linear.x = 0; //speed value m/s
                                        move.angular.z = -0.3;
                                        movement_pub.publish(move);

                                        ros::spinOnce();
                                        rate.sleep();
                                    }
                                    //move forward again
                                    ROS_INFO("move forward");
                                    ros::Time start2 = ros::Time::now();
                                    while(ros::ok() && current_pose.y > -1.5)
                                    {
                                        geometry_msgs::Twist move;
                                        //velocity controls
                                        move.linear.x = 0.2; //speed value m/s
                                        move.angular.z = 0;
                                        movement_pub.publish(move);

                                        ros::spinOnce();
                                        rate.sleep();
                                    }

                                    // just stop
                                    while(ros::ok()) {
                                        geometry_msgs::Twist move;
                                        move.linear.x = 0;
                                        move.angular.z = 0;
                                        movement_pub.publish(move);

                                        ros::spinOnce();
                                        rate.sleep();



