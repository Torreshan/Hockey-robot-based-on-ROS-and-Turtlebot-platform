#include "collision_prevention.h"

collision_prevention::collision_prevention()
{
    this->correct_theta_rad = 0.0;
    this->ang_z = 0.0;
    this->dodge_ang_z = 0.0;
    this->forward_speed_mps = 0.5;
    this->decision = STOP;
    this->previous_decision = F;
    this->makingDecision = false;
    this->movingForward = true;

    this->mid_index = 0;
    this->right_index = 329;
    this->left_index = 30;
    this->right_front_index = 349;
    this->left_front_index = 10;
    this->rL_index = left_index + 1;
    this->lL_index = rL_index + 15;
    this->lR_index = right_index - 1;
    this->rR_index = lR_index - 15;

    this->driver_status = IDLE;
    this->previous_status = IDLE;
    this->approaching = false;
    this->pg_info_set = false;

    this->navi_angz = INFINITY;
    this->navi_linx = INFINITY;
    this->fronted = false;
}

// Process the incoming laser scan message
void collision_prevention::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    if (this->driver_status == IDLE) return;
    Range closest_front_range = getClosestRange(scan, this->right_front_index, this->left_front_index);
    Range closest_range = getClosestRange(scan, this->right_index, this->left_index);

    // Evaluate left side reservoir
    Reservoir left_rsv = getReservoir(scan, this->lL_index, this->rL_index);
    // Evaluate right side reservoir
    Reservoir right_rsv = getReservoir(scan, this->lR_index, this->rR_index);

    float t_approach;

    makeDecision(left_rsv,
                 right_rsv,
                 closest_front_range,
                 closest_range,
                 this->mid_index);

    //ROS_INFO_STREAM("CFR: " << closest_front_range.range << " CR: " << closest_range.range);
    //ROS_INFO_STREAM("LR: " << left_rsv.avg_range << " RR: " << right_rsv.avg_range);

    switch (this->decision) {
    case F:
        
        break;
    case L:
        this->correct_theta_rad = abs(left_rsv.max_range_index - closest_range.index) * ANGLE_INCREMENT;
        this->correct_theta_rad = fmin(2*M_PI - this->correct_theta_rad, this->correct_theta_rad);
        t_approach = (closest_range.range / this->forward_speed_mps);
        this->ang_z = this->correct_theta_rad / t_approach + 0.1;
        break;
    case R:
        this->correct_theta_rad = abs(right_rsv.max_range_index - closest_range.index) * ANGLE_INCREMENT;
        this->correct_theta_rad = fmin(2*M_PI - this->correct_theta_rad, this->correct_theta_rad);
        t_approach = (closest_range.range / this->forward_speed_mps);
        this->ang_z = - this->correct_theta_rad / t_approach - 0.1;
        break;
    case AGGRESIVE_L:
        this->correct_theta_rad = abs(left_rsv.max_range_index - closest_range.index) * ANGLE_INCREMENT;
        this->correct_theta_rad = fmin(2*M_PI - this->correct_theta_rad, this->correct_theta_rad);
        t_approach = (closest_range.range / this->forward_speed_mps);
        if (t_approach < 0.5) {
            this->ang_z = M_PI / 2;
        } else {
            //this->ang_z = this->correct_theta_rad / t_approach + 0.2;
            this->ang_z = M_PI / 3;

        }
        break;
    case AGGRESIVE_R:
        this->correct_theta_rad = abs(right_rsv.max_range_index - closest_range.index) * ANGLE_INCREMENT;
        this->correct_theta_rad = fmin(2*M_PI - this->correct_theta_rad, this->correct_theta_rad);
        t_approach = (closest_range.range / this->forward_speed_mps);
        if (t_approach < 0.5) {
            this->ang_z = - M_PI / 2;
        } else {
            
            this->ang_z = - M_PI / 3;

        }
        break;
    case STOP:
        if (closest_range.index > 180 && this->previous_decision != STOP){
            this->ang_z = - M_PI / 3;
        } else {
            this->ang_z = M_PI / 3;
        }
        break;
    default:
        break;
    }

    this->previous_decision = decision;
    return;
}

bool collision_prevention::driveSrvCallback(turtle_vis::call_driver::Request &req,
                                            turtle_vis::call_driver::Response &res)
{
    if (this->navi_linx == INFINITY && this->navi_angz == INFINITY) {
        ROS_INFO_STREAM("CALL TURTLEVIS POSE SERVICE FIRST!");
        res.success = false;
        return true;
    }

    switch (req.task) {
    case 0:
        ROS_INFO_STREAM("======Driver: ROGER, TO CONE======");

        if (this->pg_info_set){
            if (((this->desired_x > 0) && (this->desired_x > odom_x_max_outbound)) ||
                    ((this->desired_x < 0) && (this->desired_x< odom_x_min_outbound)) ||
                    (this->desired_y > odom_y_outbound)) {
                ROS_INFO_STREAM("======OB " << this->desired_x << " " << this->desired_y << " ======");
                this->driver_status = OUTBOUND;
            } else this->driver_status = GOTOCONE;
        } else {
            this->driver_status = GOTOCONE;
        }
        break;
    case 1:
        ROS_INFO_STREAM("======Driver: ROGER, TO GOAL======");
        /// Set head toward goal
        this->fronted = false;
        this->driver_status = GOTOGOAL;
        break;
    case 2:
        this->driver_status = EXITGOAL;
        break;
    case -1:
        this->driver_status = IDLE;
        break;
    }
    res.success = true;
    return true;
}

bool collision_prevention::pgInfoCallback(turtle_vis::pg_info_transfer::Request &req,
                                          turtle_vis::pg_info_transfer::Response &res)
{
    this->pg_info_set = true;
    this->A = req.a;
    this->B = req.b;
    this->odom_y_outbound = req.b/2;
    ROS_INFO_STREAM("======Y-OB " << odom_y_outbound << " ======");

    this->odom_x_min_outbound = - req.a / 4 - 0.15;
    this->odom_x_max_outbound = 3 * req.a + odom_x_min_outbound;
    ROS_INFO_STREAM("======X-OB " << odom_x_max_outbound << " ======");

    res.success = true;
    return true;
}

bool collision_prevention::pauseCallback(std_srvs::Empty::Request &req,
                                         std_srvs::Empty::Response &res)
{
    this->driver_status = IDLE;
    this->previous_status = IDLE;
    return true;
}

void collision_prevention::poseReached()
{
    this->previous_status = driver_status;
    this->driver_status = FINISHED;
    return;
}

void collision_prevention::switchMovingDirection()
{
    if (this->movingForward) {
        this->mid_index = 0;
        this->right_index = 329;
        this->right_front_index = 349;
        this->left_index = 30;
        this->left_front_index = 10;

        this->rL_index = left_index + 1;
        this->lL_index = rL_index + 15;
        this->lR_index = right_index - 1;
        this->rR_index = lR_index - 15;
    } else {
        this->mid_index = 179;
        this->left_index = mid_index + 30;
        this->right_index = mid_index - 30;
        this->left_front_index = mid_index + 10;
        this->right_front_index = mid_index - 10;

        this->rL_index = left_index + 1;
        this->lL_index = rL_index + 15;
        this->lR_index = right_index - 1;
        this->rR_index = lR_index - 15;
    }
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
        if (approaching) this->decision = F;
        else this->decision = STOP;
    } else if (closest_front_range.range > left_rsv.avg_range && closest_front_range.range > right_rsv.avg_range) {
        if (ROS_INFO_ON) ROS_INFO_STREAM("F");
        this->decision = F;
    } else if (left_rsv.avg_range < right_rsv.avg_range && closest_range.range > this->MIN_PROXIMITY_RANGE_M * 1.4) {
        if (ROS_INFO_ON) ROS_INFO_STREAM("L but F");
        this->decision = F;
    } else if (left_rsv.avg_range > right_rsv.avg_range && closest_range.range > this->MIN_PROXIMITY_RANGE_M * 1.4) {
        if (ROS_INFO_ON) ROS_INFO_STREAM("R but F");
        this->decision = F;
    } else if ( closest_front_range.range < 1.4 * this->MIN_PROXIMITY_RANGE_M && closest_front_range.range > this->MIN_PROXIMITY_RANGE_M ) {
        if (closest_front_range.index < 180) {
            if (ROS_INFO_ON) ROS_INFO_STREAM("AR");
            if (approaching) this->decision = F;
            else this->decision = AGGRESIVE_R;

        } else {
            if (ROS_INFO_ON) ROS_INFO_STREAM("AL");
            if (approaching) this->decision = F;
            else this->decision = AGGRESIVE_L;
        }
    }
    return;
}

Range collision_prevention::getClosestRange(const sensor_msgs::LaserScan::ConstPtr& scan,
                                            const int &right_index,
                                            const int &left_index)
{
    Range range;
    range.range = scan->ranges[right_index];
    range.index = right_index;

    if (left_index - right_index < 0) {
        for (int curr_index = right_index + 1; curr_index < 359; curr_index++) {
            if (scan->ranges[curr_index] < range.range) {
                range.range = scan->ranges[curr_index];
                range.index = curr_index;
              }
        }

        for  (int curr_index = 0; curr_index < left_index; curr_index++) {
            if (scan->ranges[curr_index] < range.range) {
                range.range = scan->ranges[curr_index];
                range.index = curr_index;
              }
        }
    } else {
        for (int curr_index = right_index + 1; curr_index <= left_index; curr_index++) {
            if (scan->ranges[curr_index] < range.range) {
                range.range = scan->ranges[curr_index];
                range.index = curr_index;
              }
        }
    }

    return range;
}

geometry_msgs::Twist collision_prevention::getVelMsg(const float &lin_x,
                                                     const float &ang_z)
{
    this->navi_linx = lin_x;
    this->navi_angz = ang_z;

    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = lin_x;
    vel_msg.angular.z = ang_z;

    switch (this->driver_status) {

    case OUTBOUND:
        vel_msg.linear.x = 0.001;
        vel_msg.angular.z = M_PI / 4;
        break;

    //////////////////////////////////////////////

    case IDLE:
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        return vel_msg;

    //////////////////////////////////////////////

    case GOTOCONE:
        if (lin_x < 0 && this->movingForward) {
            this->movingForward = false;
            switchMovingDirection();
        } else if (lin_x > 0 && !this->movingForward) {
            this->movingForward = true;
            switchMovingDirection();
        }

        if (this->decision == STOP) {
            vel_msg.angular.z = this->ang_z;
            vel_msg.linear.x = 0;
        } else if (this->decision == AGGRESIVE_R || this->decision == AGGRESIVE_L) {
            if(this->movingForward) {
                vel_msg.angular.z = this->ang_z;
                vel_msg.linear.x = this->forward_speed_mps;
            } else {
                vel_msg.angular.z = - this->forward_speed_mps;
                vel_msg.linear.x = this->ang_z;
            }
        } else {
            float lin_x_;
            float ang_z_;
            if (lin_x > 0) {
                lin_x_ = fmax(lin_x, 0.2);
            } else {
                lin_x_ = fmin(lin_x, -0.2);
            }
            if (ang_z > 0) {
                ang_z_ = fmax(ang_z, 0.3);
            } else {
                ang_z_ = fmin(ang_z, -0.3);
            }
            vel_msg.linear.x = lin_x_;
            vel_msg.angular.z = ang_z_;
        }
        break;

    //////////////////////////////////////////////

    case GOTOGOAL:
        float lin_x_;
        float ang_z_;

        if (lin_x > 0) {
            lin_x_ = fmax(lin_x, 0.2);
        } else {
            lin_x_ = fmin(lin_x, -0.2);
        }
        if (ang_z > 0) {
            ang_z_ = fmax(ang_z, 0.3);
        } else {
            ang_z_ = fmin(ang_z, -0.3);
        }
        vel_msg.linear.x = lin_x_;
        vel_msg.angular.z = ang_z_;
        break;

    //////////////////////////////////////////////

    case EXITGOAL:
        vel_msg.linear.x = -0.333;
        vel_msg.angular.z = M_PI / 2;
        return vel_msg;

    //////////////////////////////////////////////

    case FINISHED:
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        return vel_msg;
    }

    if (vel_msg.linear.x > 0) vel_msg.linear.x = fmin(vel_msg.linear.x, 0.7);
    else if (vel_msg.linear.x < 0) vel_msg.linear.x = fmax(vel_msg.linear.x, -0.7);
    if (vel_msg.angular.z > 0) vel_msg.angular.z = fmin(vel_msg.angular.z, 0.7);
    else if (vel_msg.angular.z < 0) vel_msg.angular.z = fmax(vel_msg.angular.z, -0.7);
    return vel_msg;
}

Reservoir collision_prevention::getReservoir(const sensor_msgs::LaserScan::ConstPtr& scan,
                                                                    const int &left_index,
                                                                    const int &right_index)
{
    Reservoir rsv;
    rsv.max_range = scan->ranges[right_index];
    rsv.max_range_index = right_index;
    rsv.avg_range = 0;
    int count = 0;
    for (int curr_index = right_index + 1; curr_index < left_index; curr_index++) {
        if (scan->ranges[curr_index] == INFINITY) continue;
        rsv.avg_range += scan->ranges[curr_index];
        count++;
        if (scan->ranges[curr_index] > rsv.max_range) {
            rsv.max_range = scan->ranges[curr_index];
            rsv.max_range_index = curr_index;
        }
    }

    if (count == 0) {
        rsv.avg_range = 0;
        rsv.max_range = 0;
        rsv.max_range_index = 0;
        return rsv;
    }
    rsv.avg_range /= count;
    return rsv;
}
