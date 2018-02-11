#include <QTimer>
#include <QWidget>
#include <QApplication>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include "main_state_machine/call_driver.h"
#include "main_state_machine/send_desired_pose.h"
#include "main_state_machine/color_seg_finished.h"
#include "main_state_machine/pose_transfer.h"
#include "main_state_machine/pg_info_transfer.h"
#include "Controller.h"


/** This is the state machine that controls individual nodes. Based on the current and the last state, the
 * state machine calls a particular node via rosservice to perform the corresponding task. Once a node finished it's job,
 * it will calls the state machine that he is finished (also via rosservice), and the ladder will enter the next state
 * and makes the next call.
 *
 * Reason for using rosservice:
 * The purpose of implementing this state machine is that we want to ensure the execution order of the individual tasks.
 * By using rosservice, it's response reports that a particular task is executed succesfully, so we can enter the next state without any worry.
 * By daisy-chaining service callbacks with service clients, we ensure the execution order.
 * -- Class written and maintained by Daoping Wang
 * */

// An enum of all possible states
enum State {
    INIT,           // Initial state
    CONNECT,        // Connecting to Angelina
    WAIT_DS,        // Waiting for playground detection start signal
    PG_IDENT,       // Detection A, B and team color
    WAIT_GS,        // Waiting for game start signal
    SEARCH_CONE,    // Searching a puck
    GOTO_CONE,      // Driving to that puck
    CATCH_CONE,     // Pick up the puck
    GOTO_GOAL,      // Push the puck into enemy goal
    FINISHED,       // All three pucks are done, we are finished
    PAUSED          // Angelina calling us to stop moving
};

// The struct "State_machine" saves the current state, the last state and the number of goals.
struct State_machine {
    State current_state;
    State last_state;
    int num_goal;
};

// Hermes referee functionalities
Controller controller;

// Bunch of ros service clients for calling ros nodes to work
ros::ServiceClient ab_ident_client;
ros::ServiceClient color_ident_client;
ros::ServiceClient color_seg_client;
ros::ServiceClient from2dto3d_client;
ros::ServiceClient finish_color_seg_client;
ros::ServiceClient driver_command_client;
ros::ServiceClient driver_navi_client;
ros::ServiceClient hockey_catch_client;
ros::ServiceClient driver_pg_info_client;
ros::ServiceClient catch_pg_info_client;

// The state machine saving current and last state, and the number of goals
State_machine sm;

// Declare some reusable messages
std_srvs::Empty empty_srv;
std_srvs::SetBool bool_srv;
main_state_machine::send_desired_pose navi_srv;
main_state_machine::send_desired_pose goal_location_srv;
main_state_machine::call_driver tocone_srv;
main_state_machine::call_driver togoal_srv;
main_state_machine::call_driver exit_srv;

// Declare the format of the play court
float A = 1.2;
float B = 4;
double ab_ratio;
int color;
State paused_state;


// ROS service callbacks. most of these are daisy-chained with a consecutive call.

/**
 * @brief ab_ident_finished_callback is called when lidar_processing finished it's detection job.
 * @param request contains the A and B value detected by the lidar_processing node.
 * @param response a boolean
 * @return
 */
bool ab_ident_finished_callback(main_state_machine::pg_info_transfer::Request& request,
                                main_state_machine::pg_info_transfer::Response& response) {

    ROS_INFO_STREAM("======AB identified======");
    A = request.a;
    B = request.b;
    response.success = true;

    // Calculate the ABratio
    ab_ratio = A / B;

    // Let the Hermes referee tell Angelina our ABratio
    controller.tellABRatio(ab_ratio);

    // Call the next node to detect our team color!
    color_ident_client.call(empty_srv);
    return true;
}

/**
 * @brief color_ident_finished_callback is called when we detected our team color.
 * @param request contains the detected team color, 0 for blue, 1 for yellow
 * @param response a boolean, not used
 * @return
 */
bool color_ident_finished_callback(main_state_machine::pg_info_transfer::Request& request,
                                   main_state_machine::pg_info_transfer::Response& response){

    ROS_INFO_STREAM("======Color identified " << request.color << "======");
    color = request.color;
    TeamColor c;

    // Tell Angelina our color
    switch (request.color) {
    case 0:
        c = blue;
        controller.tellTeamColor(c);
        break;
    case 1:
        c = yellow;
        controller.tellTeamColor(c);
        break;
    }

    ///// 0 FOR BLUE, 1 FOR YELLOW
    // Calculate the position of the enemy goal
    goal_location_srv.request.a = 2.5 * A - 0.55;
    goal_location_srv.request.b = 0;
    goal_location_srv.request.c = 0;

    main_state_machine::pg_info_transfer srv;
    srv.request.a = A;
    srv.request.b = B;
    srv.request.color = color;

    // Tell the robot_driver node about A and B
    driver_pg_info_client.call(srv);

    // Tell hockeysearch node about A and B
    catch_pg_info_client.call(srv);

    // Enter the next state: Wait for game start signal
    sm.current_state = WAIT_GS;
    sm.last_state = PG_IDENT;
    response.success = true;
    return true;
}

/**
 * @brief color_seg_finished_callback get called when the segmentation node has found a puck and it's location
 * @param request position of the puck (odometry)
 * @param response a boolean
 * @return
 */
bool color_seg_finished_callback(main_state_machine::color_seg_finished::Request& request,
                                 main_state_machine::color_seg_finished::Response& response){

    navi_srv.request.a = request.x;
    navi_srv.request.b = request.y;
    navi_srv.request.c = request.theta;

    // Tell segmentation node to stop working
    finish_color_seg_client.call(empty_srv);

    // Tell robot_driver to drive us to the puck location
    driver_navi_client.call(navi_srv);
    driver_command_client.call(tocone_srv);

    response.success = true;

    // Enter the corresponding state, which is driving towards puck
    sm.current_state = GOTO_CONE;
    sm.last_state = SEARCH_CONE;
    return true;
}

/**
 * @brief driver_tocone_finished_callback is called when the robot_driver node says that we are in front of the puck
 * @param request empty
 * @param response empty
 * @return
 */
bool driver_tocone_finished_callback(std_srvs::Empty::Request& request,
                                     std_srvs::Empty::Response& response){
    ROS_INFO_STREAM("======REACHED CONE LOCATION======");

    // Tells hockeysearch node to start working, pick up the puck
    hockey_catch_client.call(empty_srv);

    // Enter next state
    sm.current_state = CATCH_CONE;
    sm.last_state = GOTO_CONE;
    return true;
}

/**
 * @brief driver_togoal_finished_callback is called when the robot_driver node says that we are at enemy's goal
 * @param request empty
 * @param response empty
 * @return
 */
bool driver_togoal_finished_callback(std_srvs::Empty::Request& request,
                                     std_srvs::Empty::Response& response){
    ROS_INFO_STREAM("======REACHED GOAL======");

    // Goal add 1
    sm.num_goal += 1;

    // Tell Angelina we did a goal
    controller.reportGoal();

    // Check how many goals are done. If 3, tell Angelina we are finished. If not 3, change the position of the enemy goal,
    // then call segmentation to find the next puck.
    if (sm.num_goal == 3) {
        sm.current_state = FINISHED;
        sm.last_state = GOTO_GOAL;
    }
    else {
        color_seg_client.call(bool_srv);
        from2dto3d_client.call(empty_srv);
        sm.current_state = SEARCH_CONE;
        sm.last_state = GOTO_GOAL;
    }
    return true;
}

/**
 * @brief target_outbound_callback is called if the robot_driver says that the target location is outbound
 * @param request empty
 * @param response empty
 * @return
 */
bool target_outbound_callback(std_srvs::Empty::Request& request,
                              std_srvs::Empty::Response& response) {
    ROS_INFO_STREAM("======TARGET OUTBOUND======");

    // Call segmentation to find a new puck
    color_seg_client.call(bool_srv);
    from2dto3d_client.call(empty_srv);

    // Enter the next state
    sm.current_state = SEARCH_CONE;
    sm.last_state = SEARCH_CONE;
    return true;
}

/**
 * @brief driver_exit_finished_callback DEPRECATED
 * @param request
 * @param response
 * @return
 */
bool driver_exit_finished_callback(std_srvs::Empty::Request& request,
                                   std_srvs::Empty::Response& response) {
    ROS_INFO_STREAM("======EXITED======");

    sm.current_state = SEARCH_CONE;
    sm.last_state = GOTO_GOAL;
    return true;
}

/**
 * @brief hockey_catch_finished_callback is called when we caught a puck
 * @param request empty
 * @param response empty
 * @return
 */
bool hockey_catch_finished_callback(std_srvs::Empty::Request& request,
                                    std_srvs::Empty::Response& response){
    ROS_INFO_STREAM("======CONE CAPTURED======");

    // Drive to enemy goal, position based on the number of goals we have reached
    switch (sm.num_goal) {
    case 0:
        goal_location_srv.request.b = 0;
        break;
    case 1:
        goal_location_srv.request.b = 0.3;
        break;
    case 2:
        goal_location_srv.request.b = -0.3;
        break;
    }

    driver_navi_client.call(goal_location_srv);
    driver_command_client.call(togoal_srv);

    // Enter the next state
    sm.current_state = GOTO_GOAL;
    sm.last_state = CATCH_CONE;
    return true;
}

/**
 * @brief hockey_catch_failed_callback is called if hockeysearch node failed to pick up the puck
 * @param request empty
 * @param response empty
 * @return
 */
bool hockey_catch_failed_callback(std_srvs::Empty::Request& request,
                                  std_srvs::Empty::Response& response){
    ROS_INFO_STREAM("======CAPTURE FAILED======");

    // Calls the segmentation node to find a new puck
    color_seg_client.call(bool_srv);
    from2dto3d_client.call(empty_srv);

    // Enter the searching state
    sm.current_state = SEARCH_CONE;
    sm.last_state = CATCH_CONE;
    return true;
}

int main(int argc, char **argv)	{
    QApplication app(argc, argv);
    ros::init(argc, argv, "main_state_machine");
    ros::NodeHandle nh;

    // Advertise a bunch of services. When a node finished his job, he will call the corresponding service
    ros::ServiceServer ab_ident_finished = nh.advertiseService("ab_ident_finished",
                                                               ab_ident_finished_callback);
    ros::ServiceServer color_ident_finished = nh.advertiseService("color_ident_finished",
                                                                 color_ident_finished_callback);
    ros::ServiceServer color_seg_finished = nh.advertiseService("color_seg_finished",
                                                                                 color_seg_finished_callback);
    ros::ServiceServer driver_tocone_finished = nh.advertiseService("driver_tocone_finished",
                                                                                     driver_tocone_finished_callback);
    ros::ServiceServer driver_togoal_finished = nh.advertiseService("driver_togoal_finished",
                                                                                     driver_togoal_finished_callback);
    ros::ServiceServer target_outbound_srv = nh.advertiseService("target_outbound",
                                                                 target_outbound_callback);
    ros::ServiceServer hockey_catch_finished = nh.advertiseService("catch_finished",
                                                                                    hockey_catch_finished_callback);
    ros::ServiceServer hockey_catch_failed = nh.advertiseService("catch_failed",
                                                                 hockey_catch_failed_callback);

    ros::ServiceServer driver_exit_finished = nh.advertiseService("driver_exit_finished",
                                                                  driver_exit_finished_callback);

    // Initialize a bunch of service clients for calling the nodes to perform different tasks
    driver_pg_info_client = nh.serviceClient<main_state_machine::pg_info_transfer>("driver_pg_info");
    ab_ident_client = nh.serviceClient<std_srvs::Empty>("start_measure_ab");
    color_ident_client = nh.serviceClient<std_srvs::Empty>("start_measure_color");
    color_seg_client = nh.serviceClient<std_srvs::SetBool>("start_to_color");
    driver_command_client = nh.serviceClient<main_state_machine::call_driver>("call_driver");
    driver_navi_client = nh.serviceClient<main_state_machine::send_desired_pose>("TurtlePose");
    hockey_catch_client = nh.serviceClient<std_srvs::Empty>("catching_state");
    from2dto3d_client = nh.serviceClient<std_srvs::Empty>("from2dto3dwork");
    finish_color_seg_client = nh.serviceClient<std_srvs::Empty>("finish_to_color");
    catch_pg_info_client = nh.serviceClient<main_state_machine::pg_info_transfer>("catch_pg_info");

    ros::ServiceClient pause_driver_client = nh.serviceClient<std_srvs::Empty>("driver_pause");
    ros::ServiceClient pause_catch_client = nh.serviceClient<std_srvs::Empty>("catching_state");

    tocone_srv.request.task = 0;
    togoal_srv.request.task = 1;
    exit_srv.request.task = 2;

    // Initialization finished, start connecting to Angelina
    sm.current_state = CONNECT;
    sm.last_state = INIT;
    sm.num_goal = 0;

    ros::Rate rate(100);
    controller.start_alive_timer();
    ROS_INFO_STREAM("======MSM: START CONNECTING======");
    controller.connectToServer("127.0.0.1", 10000);

    // Enter the while loop, do things based on the current and the last state we are in.
    while (ros::ok()) {
        app.processEvents();
        if (controller.paused) {
            sm.last_state = sm.current_state;
            sm.current_state = PAUSED;
            controller.paused = false;
        }

        // Based on current state, we do some things.
        switch (sm.current_state) {
        case CONNECT:
        {
            // Check if the connection is established
            if (controller.isConnected && !controller.need_reconnect) {
                ROS_INFO_STREAM("======MSM: CONNECTION SUCCESS======");
                controller.reportReady();
                sm.last_state = CONNECT;
                sm.current_state = WAIT_DS;
            } else if (controller.need_reconnect &&!controller.isConnected) {
                ROS_INFO_STREAM("======MSM: CONNECTION FAILED======");
                controller.connectToServer("127.0.0.1", 12000);
            }
            break;
        }
        case WAIT_DS:
        {
            // Check if the detection start signal is sent
            if (controller.de_started) {
                sm.current_state = PG_IDENT;
                sm.last_state = WAIT_DS;
                ab_ident_client.call(empty_srv);
                controller.de_started = false;
                ROS_INFO_STREAM("======MSM: Detection started======");

            }
            break;
        }
        case WAIT_GS:
        {
            // Check if the game is started, and if Angelina has told us the play court infomation.
            if (controller.ab_arrived) {
                A = controller.true_a;
                B = controller.true_b;
                main_state_machine::pg_info_transfer ab_srv;
                ab_srv.request.a = controller.true_a;
                ab_srv.request.b = controller.true_b;
                ab_srv.request.color = color;
                driver_pg_info_client.call(ab_srv);
                goal_location_srv.request.a = 2.5 * A - 0.55;
                goal_location_srv.request.b = 0;
                goal_location_srv.request.c = 0;
                controller.ab_arrived = false;
                ROS_INFO_STREAM("======MSM: ab received======");
            }
            if (controller.wrong_color) {
                ROS_INFO_STREAM("======Color Wrong======");
                main_state_machine::pg_info_transfer color_srv;
                color_srv.request.a = controller.true_a;
                color_srv.request.b = controller.true_b;
                switch (controller.US_COLOR) {
                case yellow:
                    color = 1;
                    break;
                case blue:
                    color = 0;
                    break;
                }
                color_srv.request.color = color;
                catch_pg_info_client.call(color_srv);
                controller.wrong_color = false;
                ROS_INFO_STREAM("======MSM: color received======");
            }
            // Enter the searching puck state once the start game signal is received
            if (controller.game_started) {
                switch (color) {
                case 0:
                    bool_srv.request.data = false;
                    break;
                case 1:
                    bool_srv.request.data = true;
                    break;
                }
                color_seg_client.call(bool_srv);
                from2dto3d_client.call(empty_srv);
                controller.game_started = false;
                sm.current_state = SEARCH_CONE;
                sm.last_state = WAIT_GS;
                ROS_INFO_STREAM("======MSM: Game started======");

            }
            break;
        }
        case PAUSED:
        {
            // If Angelina calls stop, we call the currently working node to stop.
            ROS_INFO_STREAM("======MSM: paused======");
            switch (sm.last_state) {
            case SEARCH_CONE:{
                finish_color_seg_client.call(empty_srv);
                paused_state = sm.last_state;
                sm.last_state = PAUSED;
                break;
            }
            case GOTO_CONE: {
                pause_driver_client.call(empty_srv);
                paused_state = sm.last_state;
                sm.last_state = PAUSED;
                break;
            }
            case CATCH_CONE:{
                pause_catch_client.call(empty_srv);
                paused_state = sm.last_state;
                sm.last_state = PAUSED;
                break;
            }
            case GOTO_GOAL:{
                pause_driver_client.call(empty_srv);
                paused_state = sm.last_state;
                sm.last_state = PAUSED;
                break;
            }
            case PG_IDENT:{
                ROS_INFO_STREAM("======MSM: Detect-stage pause not implemented======");
                sm.current_state = PG_IDENT;
                sm.last_state = PAUSED;
                break;
            }
            default: {
                // Once Angelina resumes the game, we call the lately working node to start his work again.
                if (controller.game_started) {
                    switch (paused_state) {
                    case SEARCH_CONE:
                        color_seg_client.call(bool_srv);
                        from2dto3d_client.call(empty_srv);
                        controller.game_started = false;
                        sm.current_state = SEARCH_CONE;
                        sm.last_state = PAUSED;
                        break;
                    case GOTO_CONE:
                        driver_navi_client.call(navi_srv);
                        driver_command_client.call(tocone_srv);
                        controller.game_started = false;
                        sm.current_state = GOTO_CONE;
                        sm.last_state = PAUSED;
                        break;
                    case GOTO_GOAL:
                        switch (sm.num_goal) {
                        case 0:
                            goal_location_srv.request.b = 0;
                            break;
                        case 1:
                            goal_location_srv.request.b = 0.3;
                            break;
                        case 2:
                            goal_location_srv.request.b = -0.3;
                            break;
                        }

                        driver_navi_client.call(goal_location_srv);
                        driver_command_client.call(togoal_srv);
                        controller.game_started = false;
                        sm.current_state = GOTO_GOAL;
                        sm.last_state = PAUSED;
                        break;
                    case CATCH_CONE:
                        hockey_catch_client.call(empty_srv);
                        controller.game_started = false;
                        sm.current_state = CATCH_CONE;
                        sm.last_state = PAUSED;
                        break;
                    default:
                        break;
                    }
                } else if (controller.de_started) {
                    ROS_INFO_STREAM("======MSM: Detect-stage pause not implemented======");

                }
                break;
            }
            }


            break;
        }
        case FINISHED:
        {
            ROS_INFO_STREAM("======MSM: Done======");
            controller.reportDone();
            break;
        }
        default: {

            break;
        }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return app.exec();
}
