#include "landmarks.h"

Landmarks::Landmarks()
{
  // Subscribe to the simulated robot's laser scan topic
    laserSub_ = node_.subscribe("/scan", 1, &Landmarks::scanCallback, this);
    laserPub_ = node_.advertise<sensor_msgs::LaserScan> ("/scan_filter", 50);
//    featurePub_ = node_.advertise<localization::featureArrayH> ("/laser_feature", 1);
//    tfListener_.setExtrapolationLimit(ros::Duration(0.1));

}


// Process the incoming laser scan message
//void Landmarks::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
//{
//    // convert /scan to point cloud
//    sensor_msgs::PointCloud2 cloud;
//    projector_.transformLaserScanToPointCloud("/base_link" , *scan, cloud, tfListener_);
//    projector_.projectLaser(*scan, cloud);
//    point_cloud_publisher_.publish(cloud);

//}

// Process the incoming laser scan message
void Landmarks::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
//    int minIndex = floor(scan->angle_min / scan->angle_increment);
//    int maxIndex = floor(scan->angle_max / scan->angle_increment);
//    std::vector<float> scanArray = scan->ranges;
//    float lastRange = scanArray[scanArray.size() - 5];
//    // scanLength = 640, scanLength2 = 639;
//    int scanLength = scanArray.size();
//    int scanLength2 = floor((scan->angle_max - scan->angle_min) / scan->angle_increment);

//    // calculate number of nan
//    int numNan = 0;
//    for (int curIdx = 0; curIdx < scanLength; curIdx ++)
//    {
//        if (std::isnan(scanArray[curIdx]) == true)
//            numNan ++;
//    }

    // angle test
//    int minIndexTest = ceil((-30.0/180*M_PI - scan->angle_min) / scan->angle_increment);
//    int maxIndexTest = floor((30.0/180*M_PI - scan->angle_max) / scan->angle_increment);
//    ROS_INFO ("minIndex: %d, maxIndex: %d", minIndexTest, maxIndexTest);

    //minIndex = -319, maxIndex = 319, scanLength = 640
//    ROS_INFO ("minIndex %d, maxIndex %d, scan length %d", minIndex, maxIndex, scanLength2);
//    ROS_INFO ("number of nan: %d", numNan);

    // find poles and publisch
    if(debug)ROS_INFO("STARTING LASERFEATUREDETECTION");
        /* clean up */
//        feature_array.features.clear();
//        feature_array.header.frame_id = "/base_link";
//        feature_array.header.stamp = ros::Time::now();

        /* init */

    double angle_min = scan->angle_min;
    double angle_max = scan->angle_max;
    double angle_increment = scan->angle_increment;

    double prev_range = scan->ranges[0];
    double curr_range = 0;
    double angle_range = 0; //curr angle of min/maxima of same ranges

    unsigned int numRanges = scan->ranges.size(); // numRanges = 640
    double ranges[numRanges];


    //test
//    for (unsigned int i = 0; i < numRanges; i++){
//        std::cout << ranges[i];
//    }

//    std::cout << " " << std::endl;


    for(size_t i = 1; i < numRanges; i++){
        curr_range = scan->ranges[i];
        if(fabs(curr_range-prev_range)< THRESHOLD_SAME_OBJ_R){

            if(isinf(curr_range)){
                if(debug)ROS_INFO("NAN");
                //NAN or INF -> recet
                angle_range = 0;
                prev_range = curr_range;
                continue;
            }
            if(debug)ROS_INFO("SAME OBJECT!");
            //same object hurray
            angle_range += angle_increment;
        }else{
            if(debug)ROS_INFO("NEW PEAK!");
            //new peak
            //1. check prev peak
            double desired_angle_range = atan2(R_POLE, prev_range);
            if(fabs(desired_angle_range - angle_range)< THRESHOLD_SAME_OBJ_A){
                //feature detected!
                double angle_f = angle_min + (i-1)*angle_increment + M_PI; //offset 0 deg looks behind him
                double range_f = prev_range;
                if(isinf(range_f) || range_f > THRESHOLD_MAX_R){
                    //recect!
                    if(debug)ROS_INFO("NAN");
                    angle_range = 0;
                    prev_range = curr_range;
                    continue;
                }
                ROS_INFO_STREAM("FEATURE! r: " << prev_range << " index: " << i << " phi: " << angle_f);

                // laserFilter.ranges
                ranges[i] = prev_range;

                // convert range,angle to x,y
                position tmpPos;
                tmpPos.x = prev_range * cos(angle_f);
                tmpPos.y = prev_range * sin(angle_f);
                positionList.push_back(tmpPos);

            }

            //2. clean up
            angle_range = 0;
        }
        //set prev_range
        prev_range = curr_range;
        if(debug)ROS_INFO_STREAM("Range: " << curr_range);


    }


    // copy ranges to scanFilter.rangs
    ros::Time scan_time = ros::Time::now();
    scanFilter.header.stamp = scan_time;
    scanFilter.header.frame_id = "laser_frame";
    scanFilter.angle_min = angle_min;
    scanFilter.angle_max = angle_max;
    scanFilter.angle_increment = angle_increment;
    scanFilter.range_max = scan->range_max;
    scanFilter.range_min = scan->range_min;

    scanFilter.ranges.resize(numRanges);    // why resize?
    for(unsigned int i = 0; i < numRanges; ++i){
      scanFilter.ranges[i] = ranges[i];
    }


    // publish scanFilter
    laserPub_.publish(scanFilter);

    // clear ranges
    for (unsigned int i = 0; i < numRanges; i++){
        ranges[i] = 0.0;
    };

    ROS_INFO_STREAM("==============================");



}

