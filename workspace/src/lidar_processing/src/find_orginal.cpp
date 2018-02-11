#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include <lidar_processing/TwoDouble.h>
const double DETECTION_RANGE = 2;
const double CYLINDER_OFFSET = 0.03;

ros::Subscriber laser_sub;
ros::Subscriber laser_sub_angel_change;


const double angle_max = 3.141593;
const double angle_min = -3.121439;
const double angle_increment = 0.0174533;
const int num_ranges = 360;

double angle_start = M_PI;
double angle_range = M_PI/36;

const double ANGLE_OFFSET = angle_min+M_PI;

// cache scan->ranegs
int cache_idx = 1;
const int cache_idx_max = 6; // from 1 to 6, cache 5 times
double cache_ranges[360][cache_idx_max-1] = {};
double ranges_to_use[num_ranges];

double range_min;
double range_min_angle;
double range1 = 0;
double range2 = 0;
double range3 = 0;
double range4 = 0;

double angle1 = 0;
double angle2 = 0;
double angle3 = 0;
double angle4 = 0;



double A;
double B;
double C;
double a;
double b;
double c;
double H;
double winkel_1;
double winkel_2;
double winkel_3;

double measure_a;
double measure_a1;
double measure_a2;
double measure_b;

// variables for 2nd laserscan call back function

int angle_change = 1;
int flage = 0;
int angle_loop_idx = 1; // 1 - 7
int angle_loop_max = 7;
double angle_range_laser_sb_angle_change = M_PI/18;

const double angle_center1 = angle_min+1.0821;  // center 65 deg
const double angle_center2 = angle_min+1.8675;  // center 107 deg
const double angle_center3 = angle_min+4.39823;  // center 252 deg
const double angle_center4 = angle_min+5.14872;  // center 295 deg
double a_b_saver[4][2] = {} ; // min_range at 95, 107 ... deg

// some flags
bool debug = false;
bool a_b_ready = false;

// some functions
int angle2index(double angle){
    int idx = int((angle- angle_min) / angle_increment);
    return idx;
}

double index2angle(int index){
    double angle = double(angle_min + index * angle_increment);
    return angle;
}

// NOTE: 107 deg = left bottom; 95 = 2nd left bottom; 65 deg = 3rd left bottom
// 252 = right bottom, 264 = 2nd right bottom; 295 = 3rd right bottom
void changeScanRange(double center, double range){
    angle_start = center - range/2 + ANGLE_OFFSET/2;
    angle_range = range;
}

double rad2degree(double rad){
    return 57.2958 * (rad-angle_min);
}

// varibales for 1st laserscan call back function
const int angle_center1_idx = angle2index(angle_center1);
const int angle_center2_idx = angle2index(angle_center2);
const int angle_center3_idx = angle2index(angle_center3);
const int angle_center4_idx = angle2index(angle_center4);
const int angle_center_threshold_idx = 5;
const double angle_center_threshold_rad = 0.0872665;  // 5 deg

// first laser call back function
void laser_sb(const sensor_msgs::LaserScanConstPtr &scan){
//    double angle_min = scan->angle_min;
//    double angle_max = scan->angle_max;
//    double angle_increment = scan->angle_increment;
//    int num_ranges = scan->ranges.size();

    // chache scan->ranges
    int num_missing = 0;
    if(cache_idx<cache_idx_max){
        //cache
        for (size_t idx = 1; idx <= num_ranges; idx++){
            if (!isinf(scan->ranges[idx])){
                cache_ranges[idx][cache_idx] = scan->ranges[idx];
            }else{
                cache_ranges[idx][cache_idx] = 0;
            }
            if(debug) std::cout << cache_ranges[idx][cache_idx] << " ";
        };
        if(debug) std::cout << std::endl;
        if(debug) std::cout << "cache_idx = " << cache_idx << std::endl;
        cache_idx++;
    }else{
        // cal mean for cache_ranges
        int num_zero = 0;
        double sum_nonzeros = 0;
        for (size_t i = 1; i <= num_ranges; i++){
            for (size_t j = 1; j < cache_idx_max; j++){
                // num of non zero
                if (cache_ranges[i][j] == 0){
                    num_zero++;
                }else{
                    sum_nonzeros += cache_ranges[i][j];
                }
            }
            if (num_zero == cache_idx_max-1){
                num_missing ++;
            }
            // all values missing, set ranges_to_use to 0
            if (num_zero == cache_idx_max-1){
                ranges_to_use[i] = 0;
            }else{
                ranges_to_use[i] = sum_nonzeros / (cache_idx_max-1-num_zero);
            }
//            if(debug) std::cout << i << ":sum: " << sum_nonzeros <<  " num_zero: " << num_zero << " range: " << ranges_to_use[i] << "  ";
            num_zero = 0;
            sum_nonzeros = 0;
            if(debug) std::cout << ranges_to_use[i] << " ";
        }
        if(debug) std::cout << "cache_id: " << cache_idx << " num of missing values: " << num_missing << std::endl;
        num_missing = 0;
        if(debug) std::cout << std::endl;
        if(debug) std::cout << "cache_idx = " << cache_idx << std::endl;
        cache_idx = 0;
    }


    // make sure angle_start and angle_range legal
    // cache_idx == 0 -> find range_min
    if (cache_idx == 0){
        if (angle_start < scan->angle_min & angle_start > scan->angle_max){
            // illegal angel_start, do nothing
        }else{
            int idx_start = angle2index(angle_start);
            int idx_stop = angle2index(angle_start + angle_range);
            range_min = scan->range_max;
            range_min_angle = scan->angle_min;
            for (size_t idx = idx_start; idx <= idx_stop; idx++){
                if (ranges_to_use[idx]<range_min & ranges_to_use[idx] != 0){
                    range_min = ranges_to_use[idx];
                    range_min_angle = index2angle(idx);
                }
            }
            // range threshold
            if (range_min >= DETECTION_RANGE){
                range_min = 0;
                range_min_angle = angle_min;
            }
            debug = true;
            if(debug) std::cout << "index range: " << idx_start << " " << idx_stop << std::endl;
//            if(debug) std::cout << "angel range: " << rad2degree(index2angle(idx_start)) << " " << rad2degree(index2angle(idx_stop)) << std::endl;
            if(debug) std::cout << "range: " << range_min << " angle: " << rad2degree(range_min_angle) << std::endl;
            debug = false;
            /*if (flage==1)
            {
                range1 = range_min;
                angle1 = range_min_angle;
            }
            if (flage==2)
            {
                range2 = range_min;
                angle2 = range_min_angle;
            }
            if (flage==3)
            {
                range3 = range_min;
                angle3 = range_min_angle;
            }
            if (flage==4)
            {
                range4 = range_min;
                angle4 = range_min_angle;
            }

            if (range1!=0)(range2!=0)(range3!=0)(range4!=0)
            {
                range1
                range2
                range3
                range4
            }
*/
            if(!a_b_ready){
                double center = angle_start+angle_range/2-ANGLE_OFFSET/2;
                if (debug) std::cout << center << " " << range_min << std::endl;
                if (debug) std::cout << std::abs(center-angle_center1) << " " << std::abs(center-angle_center2) << " " << std::abs(center-angle_center3) << " " << std::abs(center-angle_center4) << std::endl;
                if (std::abs(center-angle_center1)<angle_center_threshold_rad){
                    a_b_saver[0][0] = range_min;
                    a_b_saver[0][1] = range_min_angle;
                }else if (std::abs(center-angle_center2)<angle_center_threshold_rad){
                    a_b_saver[1][0] = range_min;
                    a_b_saver[1][1] = range_min_angle;
                }else if (std::abs(center-angle_center3)<angle_center_threshold_rad){
                    a_b_saver[2][0] = range_min;
                    a_b_saver[2][1] = range_min_angle;
                }else if (std::abs(center-angle_center4)<angle_center_threshold_rad){
                    a_b_saver[3][0] = range_min;
                    a_b_saver[3][1] = range_min_angle;
                }
                // check if a_b_saver no zero
                int a_b_saver_num_nonzero = 0;
                for (size_t idx = 0; idx < 4; idx ++){
                    if (a_b_saver[idx][0] != 0)
                        a_b_saver_num_nonzero ++;
                }
                if (a_b_saver_num_nonzero == 4)
                    a_b_ready = true;
            }else{
                // print a_b_saver
                debug = true;
                if (debug){
                    for (size_t idx = 0; idx < 4; idx ++){
                        std::cout << a_b_saver[idx][0] << " ";
                    }
                    std::cout << "range" << std::endl;
                    for (size_t idx = 0; idx < 4; idx ++){
                        std::cout << rad2degree(a_b_saver[idx][1]) << " ";
                    }
                    std::cout << "angle" << std::endl;


                    A = a_b_saver[0][0];
                    B = a_b_saver[1][0];
                    b = a_b_saver[2][0];
                    a = a_b_saver[3][0];

                    winkel_1 = a_b_saver[1][1] - a_b_saver[0][1];
                    winkel_2 = a_b_saver[2][1] - a_b_saver[1][1];
                    winkel_3 = a_b_saver[3][1] - a_b_saver[2][1];

                    measure_a1 = sqrt(pow(A,2)+pow(B,2) - 2*A*B*cos(winkel_1));
                    measure_a2 = sqrt(pow(a,2)+pow(b,2) - 2*a*b*cos(winkel_3));
                    measure_a  = (measure_a1+measure_a2)/2;
                    measure_b = sqrt(pow(b,2)+pow(B,2) - 2*b*B*cos(winkel_2));


                    std::cout << "a1" << std::endl;
                    std::cout << measure_a1<< std::endl;
                    std::cout << "a2" << std::endl;
                    std::cout << measure_a2 << std::endl;
                    std::cout << "a" << std::endl;
                    std::cout << measure_a << std::endl;
                    std::cout << "b" << std::endl;
                    std::cout << measure_b << std::endl;



                }
            }

            // save min_range at center 95, 107 ...
            }
        cache_idx = 1;
    }

}

// second laser call back function
void laser_sb_angel_change(const sensor_msgs::LaserScanConstPtr &scan){

    // rewrite if else if to switch
    debug = false;

    switch(angle_change){
        case 1:
            if(debug) std::cout << "first command" << std::endl;
            changeScanRange(angle_center1, angle_range_laser_sb_angle_change);
            flage = 1;
        break;
        case 2:
            if(debug) std::cout << "second command" << std::endl;
            changeScanRange(angle_center2, angle_range_laser_sb_angle_change);
            flage = 2;
        break;
        case 3:
            if(debug) std::cout << "third command" << std::endl;
            changeScanRange(angle_center3, angle_range_laser_sb_angle_change);
            flage = 3;
        break;
        case 4:
            if(debug) std::cout << "fouth command" << std::endl;
            changeScanRange(angle_center4, angle_range_laser_sb_angle_change);
            flage = 4;

        break;
    }
    debug = false;

    // change command
    if(debug) std::cout << angle_loop_idx << std::endl;
    if (angle_loop_idx >= angle_loop_max){
        switch(angle_change){
            case 1:
                angle_change = 2;
                if(debug) std::cout << "change command to two" << std::endl;
            break;
            case 2:
                angle_change = 3;
                if(debug) std::cout << "change command to three" << std::endl;
            break;
            case 3:
                angle_change = 4;
                if(debug) std::cout << "change command to one" << std::endl;
            break;
            case 4:
                angle_change = 1;
                if(debug) std::cout << "change command to one" << std::endl;
            break;
        }
        angle_loop_idx = 1;
    }
    angle_loop_idx ++;

    debug = false;
}



bool measure(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    return true;
}





int main(int argc, char **argv){

    ros::init(argc,argv,"feature_node");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("measure_ab",measure);

    ROS_INFO("Ready to measure a b.");

    ros::Rate rate(30);
    int cache_idx = 1;

    // inti angel_start and angle_stop
    changeScanRange(angle_min+M_PI/2*3, M_PI/9*3);


    laser_sub = nh.subscribe("/lidarscan", 1, &laser_sb);
    laser_sub_angel_change = nh.subscribe("/lidarscan", 1, &laser_sb_angel_change);





     ros::Duration(5).sleep();

     ros::ServiceClient client_ = nh.serviceClient<lidar_processing::TwoDouble>("get_measure_ab");
     lidar_processing::TwoDouble srv;
     srv.request.A = measure_a;


     if(client_.call(srv))
     {
     ROS_INFO("Ready to get a b.");
     }
     else
     {
     ROS_INFO("Failed to call service get_measure_ab.");

     }

     ros::spin();
     return 0;
}
