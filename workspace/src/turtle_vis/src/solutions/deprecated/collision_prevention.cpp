#include "collision_prevention.h"

collision_prevention::collision_prevention(const turtleSpace::TurtleClass &turtleF)
{
    //this->keep_moving = true;
    this->correct_theta_rad = 0.0;
    this->ang_z = 0.0;
    this->dodge_ang_z = 0.0;
    this->forward_speed_mps = 0.2;
    this->decision = STOP;
    this->previous_decision = STOP;
    this->target_odom.x = 0;
    this->target_odom.y = 0;
    this->target_odom.yaw = 0;
    this->makingDecision = false;
    this->turtleF = turtleF;
}



// Process the incoming laser scan message
void collision_prevention::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    //if (!makingDecision) this->scan = scan;

    // Find the closest range between the defined minimum and maximum angles
    int min_index = ceil((this->MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
    int min_front_index = ceil((this->MIN_FRONT_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
    int max_index = floor((this->MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
    int max_front_index = floor((this->MAX_FRONT_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
    float mid_index = (max_index + min_index) / 2.0;
    float mid_front_index = (max_front_index + min_front_index) / 2.0;

    int min_left_side_index = max_index + 1;
    int max_left_side_index = min_left_side_index + floor(this->SIDE_AREA_ANGLE_RAD / scan->angle_increment);
    int max_right_side_index = min_index - 1;
    int min_right_side_index = max_right_side_index - floor(this->SIDE_AREA_ANGLE_RAD / scan->angle_increment);

    Range closest_front_range = getClosestRange(scan, min_front_index, max_front_index);
    Range closest_range = getClosestRange(scan, min_index, max_index);

    // Evaluate left side reservoir
    Reservoir left_rsv = getReservoir(scan, min_left_side_index, max_left_side_index);

    // Evaluate right side reservoir
    Reservoir right_rsv = getReservoir(scan, min_right_side_index, max_right_side_index);

    float t_approach;

    makeDecision(left_rsv,
                 right_rsv,
                 closest_front_range,
                 closest_range,
                 mid_index);

    switch (this->decision) {
    case F:
        //this->ang_z = 0;
        moveForward();
        break;
    case L:
        this->correct_theta_rad = abs(left_rsv.max_range_index - closest_range.index) * scan->angle_increment;
        this->correct_theta_rad = fmin(2*M_PI - this->correct_theta_rad, this->correct_theta_rad);
        t_approach = (closest_range.range / this->forward_speed_mps);
        this->ang_z = this->correct_theta_rad / t_approach + 0.1;
        break;
    case R:
        this->correct_theta_rad = abs(right_rsv.max_range_index - closest_range.index) * scan->angle_increment;
        this->correct_theta_rad = fmin(2*M_PI - this->correct_theta_rad, this->correct_theta_rad);
        t_approach = (closest_range.range / this->forward_speed_mps);
        this->ang_z = - this->correct_theta_rad / t_approach - 0.1;
        break;
    case AGGRESIVE_L:
        this->correct_theta_rad = abs(left_rsv.max_range_index - closest_range.index) * scan->angle_increment;
        this->correct_theta_rad = fmin(2*M_PI - this->correct_theta_rad, this->correct_theta_rad);
        t_approach = (closest_range.range / this->forward_speed_mps);
        if (t_approach < 0.5) {
            this->ang_z = - M_PI / 2;
        } else {
            this->ang_z = this->correct_theta_rad / t_approach + 0.2;
        }
        break;
    case AGGRESIVE_R:
        this->correct_theta_rad = abs(right_rsv.max_range_index - closest_range.index) * scan->angle_increment;
        this->correct_theta_rad = fmin(2*M_PI - this->correct_theta_rad, this->correct_theta_rad);
        t_approach = (closest_range.range / this->forward_speed_mps);
        if (t_approach < 0.5) {
            this->ang_z = - M_PI / 2;
        } else {
            this->ang_z = - this->correct_theta_rad / t_approach - 0.2;
        }
        break;
    case STOP:
        if (closest_range.index > mid_index && this->previous_decision != STOP){
            this->ang_z = - M_PI / 4;
        } else {
            this->ang_z = M_PI / 4;
        }
        break;
    default:
        break;
    }

    this->previous_decision = decision;
    return;
}

void collision_prevention::odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
    this->base_odom.x = odom->pose.pose.position.x;
    this->base_odom.y = odom->pose.pose.position.y;
    this->base_odom.yaw = quaternionToYaw(odom->pose.pose.orientation.x,
                                        odom->pose.pose.orientation.y,
                                        odom->pose.pose.orientation.z,
                                        odom->pose.pose.orientation.w);

    float wish_ang_z = atan2((target_odom.y - base_odom.y) , (target_odom.x - base_odom.x));
    return;
}

void collision_prevention::setTargetOdom(const float &x, const float &y, const float &yaw)
{
    this->target_odom.x = x;
    this->target_odom.y = y;
    this->target_odom.yaw = yaw;
    return;
}

void collision_prevention::makeDecision(const Reservoir &left_rsv,
                                        const Reservoir &right_rsv,
                                        const Range &closest_front_range,
                                        const Range &closest_range,
                                        const int &mid_index)
{
    if (closest_range.range < this->MIN_PROXIMITY_RANGE_M) {
        if (ROS_INFO_ON) ROS_INFO_STREAM("STOP");
        decision = STOP;
    } else if (closest_front_range.range > left_rsv.avg_range && closest_front_range.range > right_rsv.avg_range) {
        if (ROS_INFO_ON) ROS_INFO_STREAM("F");
        this->decision = F;
    } else if (left_rsv.avg_range < right_rsv.avg_range && closest_range.range > this->MIN_PROXIMITY_RANGE_M * 2) {
        if (ROS_INFO_ON) ROS_INFO_STREAM("R but F");
        this->decision = F;
        //this->decision = R;
    } else if (left_rsv.avg_range > right_rsv.avg_range && closest_range.range > this->MIN_PROXIMITY_RANGE_M * 2) {
        if (ROS_INFO_ON) ROS_INFO_STREAM("L but F");
        this->decision = F;
//        this->decision = L;
    } else if ( closest_range.range < 1.5 * this->MIN_PROXIMITY_RANGE_M && closest_range.range > this->MIN_PROXIMITY_RANGE_M ) {
        if (closest_range.index > mid_index) {
            if (ROS_INFO_ON) ROS_INFO_STREAM("AR");
            this->decision = AGGRESIVE_R;
        } else {
            if (ROS_INFO_ON) ROS_INFO_STREAM("AL");
            this->decision = AGGRESIVE_L;
        }
    }
    return;
}

Range collision_prevention::getClosestRange(const sensor_msgs::LaserScan::ConstPtr& scan,
                                            const int &min_index,
                                            const int &max_index)
{
    Range range;
    range.range = scan->ranges[min_index];
    range.index = min_index;
    for (int curr_index = min_index + 1; curr_index <= max_index; curr_index++) {
        if (scan->ranges[curr_index] < range.range) {
            range.range = scan->ranges[curr_index];
            range.index = curr_index;
          }
    }
    return range;
}

geometry_msgs::Twist collision_prevention::getVelMsg()
{
    /*// START
    if (this->previous_decision == IDLE) {
        this->makingDecision = true;

        float wish_angle_rad = atan2(y, x) / 180*M_PI;
        float wish_speed = 0.5 * sqrt(pow(x, 2) + pow(y, 2));

        int min_wish_index = ceil((wish_angle_rad + this->MIN_FRONT_ANGLE_RAD - this->scan->angle_min) / this->scan->angle_increment);
        int max_wish_index = floor((wish_angle_rad + this->MAX_FRONT_ANGLE_RAD - this->scan->angle_min) / this->scan->angle_increment);
        int mid_wish_index = ceil((min_wish_index + max_wish_index) / 2);

        int min_index = ceil((wish_angle_rad + this->MIN_SCAN_ANGLE_RAD - this->scan->angle_min) / this->scan->angle_increment);
        int max_index = floor((wish_angle_rad + this->MAX_SCAN_ANGLE_RAD - this->scan->angle_min) / this->scan->angle_increment);

        int min_left_side_index = max_wish_index + 1;
        int max_left_side_index = min_left_side_index + floor(this->SIDE_AREA_ANGLE_RAD / this->scan->angle_increment);
        int max_right_side_index = min_wish_index - 1;
        int min_right_side_index = max_right_side_index - floor(this->SIDE_AREA_ANGLE_RAD / this->scan->angle_increment);

        Range closest_wish_range = getClosestRange(this->scan, min_wish_index, max_wish_index);
        Range closest_range = getClosestRange(this->scan, min_index, max_index);
        Reservoir left_rsv = getReservoir(this->scan, min_left_side_index, max_left_side_index);
        Reservoir right_rsv = getReservoir(this->scan, min_right_side_index, max_right_side_index);

        makeDecision(left_rsv,
                     right_rsv,
                     closest_wish_range,
                     closest_range,
                     mid_wish_index);

        this->makingDecision = false;
    }



    // END
    */
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = this->ang_z;

    if (this->decision != STOP) {
        vel_msg.linear.x = this->forward_speed_mps;
    } else {
        vel_msg.linear.x = 0;
    }
    return vel_msg;
}

Reservoir collision_prevention::getReservoir(const sensor_msgs::LaserScan::ConstPtr& scan,
                                                                    const int &min_index,
                                                                    const int &max_index)
{
    Reservoir rsv;

    rsv.max_range = scan->ranges[min_index];
    rsv.max_range_index = min_index;
    rsv.avg_range = 0;
    int count = 0;
    for (int curr_index = min_index + 1; curr_index < max_index; curr_index++) {
        rsv.avg_range += scan->ranges[curr_index];
        count++;
        if (scan->ranges[curr_index] > rsv.max_range) {
            rsv.max_range = scan->ranges[curr_index];
            rsv.max_range_index = curr_index;
        }
    }
    rsv.avg_range /= count;
    return rsv;
}

float collision_prevention::quaternionToYaw(const float &x,
                                            const float &y,
                                            const float &z,
                                            const float &w)
{
    float siny = +2.0 * (w * z + x * y);
    float cosy = +1.0 - 2.0 * (y * y + z * z);
    float yaw = atan2(siny, cosy);
    return yaw;
}

void collision_prevention::moveForward() {
    Vector3d turtleTIAGOPose;
    Vector3d turtleTIAGOPose_local;
    turtleTIAGOPose<<1,0,0;    // WHY-----------------------------------------------------

    turtleTIAGOPose_local=turtleTIAGOPose;
    turtleF.turtlePose_g=turtleTIAGOPose;  // WHY-----------------------------------------------------




    ros::Time ti, tf;
    ti=ros::Time::now();

    //Proportional Gain
    Matrix3d Kp;




    //#>>>>TODO: SET GAINS

    double p_g=0.0;

    //#>>>>TODO: LOAD p_gain FROM THE ROS PARAMETER SERVER
    //ros::param::get(/*/#>>>>TODO: DEFINE PARAMETER*/,p_g);
    ros::param::get("/turtle_gains/p_gain",p_g);
    ROS_INFO_STREAM("p_g= "<<p_g);


    //Proportional Gain

    Kp<<p_g,0  ,0,
            0  ,p_g,0,
            0  ,0  ,p_g;

    ROS_INFO_STREAM("Kp= \n"<<Kp);

    Vector3d turtlePose,turtlePose_old,turtleVel;
    Vector3d error;
    double dt;

    //INITIALIZE THE TURTLE POSE
    turtlePose<<1,0,0;
    turtlePose_old=turtlePose;
    turtleVel<<0,0,0;

    //DESIRED POSE
    Vector3d turtlePose_desired_local;
    ////#>>>>TODO: INITIALIZE THE DESIRED POSE VARIABLE OF THE CLASS TURTLE
    turtleF.turtlePose_desired_g=turtlePose;
    turtlePose_desired_local=turtlePose;


    //CREATE A DESIREDPOSE MSG VARIABLE
    //turtle_vis::DesiredPose msg; //#>>>>TODO:DEFINE THE MSG TYPE  OK
    //std::cout<<"x= "<<msg.x<<"\t y= "<<msg.y<<"\t z= "<<msg.theta<<std::endl;

    geometry_msgs::Twist msg2;

    Matrix2d K;
    K<<0.5,0,
       0, 0.5;

    Matrix2d T_g;
    T_g<<0.1,0.1,
            0.1,0.1;

    float d = 0.3;
    Vector2d turtleTIAGOPose_2d,turtleTIAGOPose_2d_Now;
    Vector2d turtleTIAGOPose_desired_local_2d;

    turtleTIAGOPose_2d << 1,1;                   // Why 1 ?-----------------------------------------
    turtleTIAGOPose_desired_local_2d << 1,1;


    Vector2d error_2d;
    Vector2d turtleVel_2d;
    turtleVel_2d<<0,0;

    while (this->decision == F) {
        tf=ros::Time::now();

        dt=tf.toSec()-ti.toSec();


        ////# Get Local  Suscribe
        turtleTIAGOPose_local=turtleF.getLocalPose();

        //TEST
        std::cout<<"X: \t"<<turtleTIAGOPose_local[0]<<"Y: \t"<<turtleTIAGOPose_local[1]<<"Z: \t"<<turtleTIAGOPose_local[2]<<std::endl;
        turtleTIAGOPose_2d<<turtleTIAGOPose_local[0],turtleTIAGOPose_local[1];



        ////#>>>>TODO: Get Desired Pose from the class variable
        //turtlePose_desired_local=turtleF.CALL_THE_METHOD_TO_OBTAIN_THE_DESIRED_POSE();
        turtlePose_desired_local=turtleF.getLocalDesiredPose();

        turtleTIAGOPose_desired_local_2d<< turtlePose_desired_local[0], turtlePose_desired_local[1];


        //CONTROL
        ////#>>>>TODO:COMPUTE THE ERROR BETWEEN CURRENT POSE AND DESIRED
        //error=turtlePose_desired_local-turtlePose;
        //error=turtlePose-turtlePose_desired_local;
        error_2d=turtleTIAGOPose_desired_local_2d-turtleTIAGOPose_2d;


        // COMPUTE THE INCREMENTS
        //turtleVel=-Kp*error;
        turtleVel_2d=K*error_2d;

        T_g<< 1*cos(turtleTIAGOPose_local[2]),1*sin(turtleTIAGOPose_local[2]),
                -sin(turtleTIAGOPose_local[2])/d, cos(turtleTIAGOPose_local[2])/d;

        //-----------------------------------------------------------------------------------Put on ?

        turtleTIAGOPose_2d_Now=T_g*turtleVel_2d;




        ////#>>>>TODO:COMPUTE THE NEW TURTLE POSE
        turtlePose= turtlePose + turtleVel*dt;  //USE SIMPLE INTEGRATION

        //Publish Data
        ////#>>>>TODO:SET THE MSG VARIABLE WITH THE NEW TURTLE POSE
        //msg.x = turtlePose[0];
        //msg.y = turtlePose[1];
        //msg.theta = turtlePose[2];

        //msg2.linear.x=turtleTIAGOPose_2d_Now(0);
        //msg2.angular.z=turtleTIAGOPose_2d_Now(1);
        //commandPub.publish(msg2);

        this->forward_speed_mps = turtleTIAGOPose_2d_Now(0);
        this->ang_z = turtleTIAGOPose_2d_Now(1);

        //pub.publish(msg);
        //std::cout<<"xx = "<< turtlePose[0] << "\t yy= " << turtlePose[1] << "\t zz= " << turtlePose[2] << std::endl;

        //SET THE HISTORY
        turtlePose_old=turtlePose;
        ti=tf;

    }
}
