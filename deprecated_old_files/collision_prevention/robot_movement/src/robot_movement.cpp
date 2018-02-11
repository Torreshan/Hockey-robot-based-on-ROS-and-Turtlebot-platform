#include "robot_movement.h"

robot_movement::robot_movement()
{
    keepMoving = true;
    correctThetaRad = 0.0;
    angZ = 0.0;
    dodgeAngZ = 0.0;

    // Advertise a new publisher for the simulated robot's velocity command topic
    commandPub = node.advertise<geometry_msgs::Twist>(VEL_COMMAND_TOPIC_NAME, 10);

    laserSub = node.subscribe(LASER_TOPIC_NAME, 1, &robot_movement::scanCallback, this);
    targetSub = node.subscribe(TARGET_TOPIC_NAME, 1, &robot_movement::targetCallback, this);

}

