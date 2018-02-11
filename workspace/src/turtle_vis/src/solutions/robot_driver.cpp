#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtle_vis/myClass/TurtleClass.h>
#include <std_srvs/Empty.h>
#include "collision_prevention.h"


/** The robot_driver node drives the robot to the target location requested by the navigation node.
* This process is realized by combining TurtleClass, the class responsible for calculating the linear and
* angular speed to the target, with collision_prevention, the class that uses lidar scan messages to
* avoid obstacles along the way and plan the path.
* -- Class written and maintained by Daoping Wang
* */

int main(int argc, char	**argv)	{
    ros::init(argc, argv, "robot_driver");
    ROS_INFO("ROBOT_DRIVER_STARTED");

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

    // Service for setting playground length and width
    ros::ServiceServer pg_info_srv = nh.advertiseService("driver_pg_info",
                                                         &collision_prevention::pgInfoCallback,
                                                         &col_prevention);

    // Advertise a service for GOTO-target-location requests
    ros::ServiceServer service = nh.advertiseService("TurtlePose",
                                                     &turtleSpace::TurtleClass::getDPose,
                                                     &turtleF);

    ros::ServiceServer pause_service = nh.advertiseService("driver_pause",
                                                           &collision_prevention::pauseCallback,
                                                           &col_prevention);


    // Subscribe to /odom in order to calculate velocity
    ros::Subscriber sub = nh.subscribe("/odom",100,
                                          &turtleSpace::TurtleClass::getTiagoPose,
                                          &turtleF);

    ros::ServiceClient tocone_finished_client = nh.serviceClient<std_srvs::Empty>("driver_tocone_finished");
    ros::ServiceClient togoal_finished_client = nh.serviceClient<std_srvs::Empty>("driver_togoal_finished");
    ros::ServiceClient target_outbound_client = nh.serviceClient<std_srvs::Empty>("target_outbound");

    std_srvs::Empty empty_srv;
    turtle_vis::call_driver exit_driver_srv;
    exit_driver_srv.request.task = 2;

    ros::Rate rate(60);

    Vector3d turtleTIAGOPose;
    Vector3d turtleTIAGOPose_local;
    turtleTIAGOPose<<1,0,0;
    turtleTIAGOPose_local=turtleTIAGOPose;
    turtleF.turtlePose_g=turtleTIAGOPose;


    ros::Time ti, tf;
    ti=ros::Time::now();

    // Proportional Gain
    Matrix3d Kp;

    // Set gains
    double p_g=0.19;

    // LOAD p_gain FROM THE ROS PARAMETER SERVER
    ros::param::get("/turtle_gains/p_gain",p_g);
    ROS_INFO_STREAM("p_g= "<<p_g);


    // Proportional Gain
    Kp<<p_g,0  ,0,
            0  ,p_g,0,
            0  ,0  ,p_g;
    ROS_INFO_STREAM("Kp= \n"<<Kp);

    Vector3d turtlePose,turtlePose_old,turtleVel, turtleOdomPose;
    Vector3d error;
    double dt;

    // INITIALIZE THE TURTLE POSE
    turtlePose<<1,0,0;
    turtlePose_old=turtlePose;
    turtleVel<<0,0,0;

    // DESIRED POSE
    Vector3d turtlePose_desired_local;

    // INITIALIZE THE DESIRED POSE VARIABLE OF THE CLASS TURTLE
    turtleF.turtlePose_desired_g=turtlePose;
    turtlePose_desired_local=turtlePose;


    // CREATE A DESIREDPOSE MSG VARIABLE
    geometry_msgs::Twist msg2;

    Matrix2d K;
    K<<0.4,0,
       0, 0.4;

    Matrix2d T_g;

    float d = 0.2;
    Vector2d turtleTIAGOPose_2d,turtleTIAGOPose_2d_Now;
    Vector2d turtleTIAGOPose_desired_local_2d;
    Vector2d error_2d;
    Vector2d turtleVel_2d;

    while (ros::ok()) {
        tf=ros::Time::now();

        dt=tf.toSec()-ti.toSec();

        // Get Local  Suscribe
        turtleTIAGOPose_local=turtleF.getLocalPose();

        // TEST
        turtleTIAGOPose_2d<<turtleTIAGOPose_local[0],turtleTIAGOPose_local[1];


        // Get Desired Pose from the class variable
        turtlePose_desired_local=turtleF.getLocalDesiredPose();

        turtleTIAGOPose_desired_local_2d<< turtlePose_desired_local[0], turtlePose_desired_local[1];
        col_prevention.desired_x = turtlePose_desired_local[0];
        col_prevention.desired_y = fabs(turtlePose_desired_local[1]);


        // CONTROL
        // COMPUTE THE ERROR BETWEEN CURRENT POSE AND DESIRED
        error_2d=turtleTIAGOPose_desired_local_2d-turtleTIAGOPose_2d;

        // COMPUTE THE INCREMENTS
        turtleVel_2d=K*error_2d;

        T_g<< 1*cos(turtleTIAGOPose_local[2]),1*sin(turtleTIAGOPose_local[2]),
                -sin(turtleTIAGOPose_local[2])/d, cos(turtleTIAGOPose_local[2])/d;


        turtleTIAGOPose_2d_Now=T_g*turtleVel_2d;

        // COMPUTE THE NEW TURTLE POSE (USE SIMPLE INTEGRATION)
        turtlePose= turtlePose + turtleVel*dt;

        float lin_x= turtleTIAGOPose_2d_Now(0);
        float ang_z = turtleTIAGOPose_2d_Now(1);

        // SET THE HISTORY
        turtlePose_old=turtlePose;
        ti=tf;

        /** Pass the go-to-target velocity to col_prevention and let it either returns the velocity (if no critical collision danger),
        or returns a velocity that allows the turtle to maneuver around the obstacle. */
        geometry_msgs::Twist vel_msg = col_prevention.getVelMsg(lin_x, ang_z);


        // Depending on the current state, the robot_driver makes the decision whether to enable/disable collision prevention.
        switch (col_prevention.driver_status) {
        case OUTBOUND:
        {
            double start_time = ros::Time::now().toSec();
            double cur_time;
            do {

                vel_msg.angular.z = 0;
                vel_msg.linear.x = 0.2;
                vel_pub.publish(vel_msg);
                cur_time = ros::Time::now().toSec();
            } while ((cur_time - start_time) < 2.0);
            col_prevention.driver_status = FINISHED;
            col_prevention.previous_status = OUTBOUND;
            break;
        }
        case IDLE:
            break;
        case EXITGOAL:
        {
            // This is the state when robot_driver has finished pushing the puck into enemy's goal.
            // The robot will then drive backwards for 1 second, and turn 180 degrees.
            double start_time = ros::Time::now().toSec();
            double cur_time;
            do {
                vel_msg.angular.z = 0;
                vel_pub.publish(vel_msg);
                cur_time = ros::Time::now().toSec();
            } while ((cur_time - start_time) < 1);

            start_time = ros::Time::now().toSec();

            do {
                vel_msg.linear.x = 0;
                vel_msg.angular.z = M_PI / 3;
                vel_pub.publish(vel_msg);
                cur_time = ros::Time::now().toSec();
            } while ((cur_time - start_time) < 2.7);

            col_prevention.driver_status = FINISHED;
            col_prevention.previous_status = EXITGOAL;
            break;
        }
        case FINISHED:
        {
            // When robot is at the target location/finishes his job, the robot_driver node will report the main_state_machine.
            switch (col_prevention.previous_status) {
            case OUTBOUND: {
                ROS_INFO_STREAM("======TARGET OUTBOUND======");
                col_prevention.driver_status = IDLE;
                col_prevention.previous_status = IDLE;
                col_prevention.approaching = false;
                target_outbound_client.call(empty_srv);
                break;
            }
            case GOTOCONE:
                tocone_finished_client.call(empty_srv);
                ROS_INFO_STREAM("======CONE REACHED======");
                col_prevention.driver_status = IDLE;
                col_prevention.previous_status = IDLE;
                col_prevention.approaching = false;
                break;

            case GOTOGOAL:
            {
                ROS_INFO_STREAM("======GOAL REACHED======");
                col_prevention.driver_status = EXITGOAL;
                col_prevention.previous_status = GOTOGOAL;
                col_prevention.approaching = false;
                break;
            }
            case EXITGOAL:
                ROS_INFO_STREAM("======EXIT======");
                col_prevention.driver_status = IDLE;
                col_prevention.previous_status = IDLE;
                col_prevention.approaching = false;
                togoal_finished_client.call(empty_srv);
                break;
            }
            break;

        }
        case GOTOGOAL:
        {
            if (fabs(turtleTIAGOPose_local[2]) > 0.15 * M_PI && !col_prevention.fronted)
                vel_msg.linear.x = 0;
            else col_prevention.fronted = true;
            vel_pub.publish(vel_msg);
            if (fabs(error_2d[0]) < 0.5 && fabs(error_2d[1]) < 0.5) col_prevention.approaching = true;
            if (fabs(error_2d[0]) < 0.1 && fabs(error_2d[1]) < 0.1) col_prevention.poseReached();
            break;
        }
        default:
            vel_pub.publish(vel_msg);
            //ROS_INFO_STREAM(error_2d);
            if (fabs(error_2d[0]) < 0.5 && fabs(error_2d[1]) < 0.5) col_prevention.approaching = true;
            if (fabs(error_2d[0]) < 0.1 && fabs(error_2d[1]) < 0.1) col_prevention.poseReached();
            break;
        }

        // Get incoming messages
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
