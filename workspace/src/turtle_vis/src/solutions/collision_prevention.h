#ifndef COLLISION_PREVENTION_H
#define COLLISION_PREVENTION_H

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "turtle_vis/myClass/TurtleClass.h"
#include "turtle_vis/call_driver.h"
#include "turtle_vis/pose_transfer.h"
#include "turtle_vis/pg_info_transfer.h"

// Enum for different decisions the driver can make
enum Decision {
    F,              // Do not change the incoming velocity navigating to the target
    L,              // Drive a little bit to left
    R,              // Drive a little bit to the right
    AGGRESIVE_L,    // Make a sharp left turn
    AGGRESIVE_R,    // Make a sharp right turn
    STOP            // Stop and turn
};

// For determining the current task the state machine is telling us to perform.
enum Status {
    IDLE,           // Not working, don't move
    GOTOCONE,       // Go to puck with collision prevention
    GOTOGOAL,       // Push the puck to goal without collision prevention
    EXITGOAL,       // Exit the enemy goal
    FINISHED,       // Finished the current job
    OUTBOUND        // The target location is outside the play court
};

// Side spaces
struct Reservoir{
    float max_range;
    float avg_range;
    int max_range_index;
};

struct Range{
    float range;
    int index;
};

struct Odom{
    float x;
    float y;
    float yaw;
};

/**
 * @brief The collision_prevention class avoids obstacle based on lidar scan messages. It evaluates the space
 * on the front left and front right side, and chooses between:
 * - Don't change the speed calculated by turtleClass, which is leading the robot to the target location
 * - Maneuver around the obstacle by turning right or left while keep going
 * - Stop and turn to avoid obstacle
 * If the incoming linear velocity is < 0, the collision_prevention class will know that we are driving
 * backwards and changes the lidar scan indices accordingly.
 * -- Class written and maintained by Daoping Wang
 */
class collision_prevention
{
public:
    collision_prevention();

    // Fix constants
    const static bool ROS_INFO_ON = false;
    const static double MIN_SCAN_ANGLE_RAD = -40.0/180*M_PI;
    const static double MAX_SCAN_ANGLE_RAD = 40.0/180*M_PI;
    const static double MIN_FRONT_ANGLE_RAD = -12.0/180*M_PI;
    const static double MAX_FRONT_ANGLE_RAD = 12.0/180*M_PI;
    const static float MIN_PROXIMITY_RANGE_M = 0.5;
    const static float SIDE_AREA_ANGLE_RAD = 10.0/180*M_PI;
    const static float ANGLE_MIN = -3.12413907051;
    const static float ANGLE_INCREMENT = 0.0174532923847;

    // Flags for state determination
    Status driver_status;
    Status previous_status;
    bool approaching;
    bool pg_info_set;
    bool fronted;

    // Playground info
    float A;
    float B;
    float odom_x_min_outbound;
    float odom_x_max_outbound;
    float odom_y_outbound;

    // Target location
    float desired_x;
    float desired_y;

    /**
     * @brief getVelMsg returns the velocity determined by collision_prevention
     * @param lin_x incoming linear speed that leads us to the target location
     * @param ang_z incoming angular speed
     * @return The corrected velocity, or in other words, the decision made
     */
    geometry_msgs::Twist getVelMsg(const float &lin_x,
                                   const float &ang_z);

    /**
     * @brief scanCallback processes lidar scans
     * @param scan
     */
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

    /**
     * @brief driveSrvCallback for main_state_machine to call us to get working
     * @param req
     * @param res
     * @return
     */
    bool driveSrvCallback(turtle_vis::call_driver::Request &req,
                          turtle_vis::call_driver::Response &res);

    /**
     * @brief pgInfoCallback gets A and B from main_state_machine
     * @param req
     * @param res
     * @return
     */
    bool pgInfoCallback(turtle_vis::pg_info_transfer::Request &req,
                        turtle_vis::pg_info_transfer::Response &res);

    /**
     * @brief pauseCallback If Angelina calls stop and driver is working, the main_state_machine will
     * tell the driver by this service
     * @param req
     * @param res
     * @return
     */
    bool pauseCallback(std_srvs::Empty::Request &req,
                       std_srvs::Empty::Response &res);
    void poseReached();

private:
    // A bunch of indices for lidar scan processes
    float mid_index;
    int right_index;
    int left_index;
    int right_front_index;
    int left_front_index;
    int lL_index;
    int rL_index;
    int lR_index;
    int rR_index;

    bool makingDecision;
    bool movingForward;

    double forward_speed_mps;
    float correct_theta_rad;
    float ang_z;
    float dodge_ang_z;

    float navi_linx;
    float navi_angz;

    Decision decision;
    Decision previous_decision;

    void switchMovingDirection();
    void makeDecision(const Reservoir &left_rsv,
                      const Reservoir &right_rsv,
                      const Range &closest_front_range,
                      const Range &closest_range,
                      const int &mid_index);
    Range getClosestRange(const sensor_msgs::LaserScan::ConstPtr& scan,
                          const int &right_index,
                          const int &left_index);
    Reservoir getReservoir(const sensor_msgs::LaserScan::ConstPtr& scan,
                           const int &right_index,
                           const int &left_index);
};

#endif // COLLISION_PREVENTION_H
