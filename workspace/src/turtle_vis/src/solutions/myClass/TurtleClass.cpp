#include<turtle_vis/myClass/TurtleClass.h>
#include <nav_msgs/Odometry.h>



namespace turtleSpace {

TurtleClass::TurtleClass()
{
    //#>>>>TODO: INITIALIZE MEMBER VARIABLES
    //constructor
    turtlePose_g.setZero();
    turtlePose_desired_g.setZero();
    count_mutex = PTHREAD_MUTEX_INITIALIZER;
}
TurtleClass::~TurtleClass()
{


}

void TurtleClass::getPose(const turtle_vis::DesiredPose::ConstPtr &msg)
{
    pthread_mutex_lock( &this->count_mutex );
    //#>>>>TODO: COPY THE MSG TO A LOCAL VARIABLE
//    float x_t = msg->x;
//    float y_t = msg->y;
//    float theta_t = msg->theta;

    this->turtlePose_g(0) = msg->x;
    this->turtlePose_g(1) = msg->y;
    this->turtlePose_g(2) = msg->theta;

    pthread_mutex_unlock( &this->count_mutex );
    //#>>>>TODO:PLOT THE OBTAINED DATA
    std::cout<< this->turtlePose_g(0) <<"   "<< this->turtlePose_g(1)  <<"  "<< this->turtlePose_g(2) <<std::endl;
}

bool TurtleClass::getDPose(turtle_vis::send_desired_pose::Request &req, turtle_vis::send_desired_pose::Response &res)
{
    pthread_mutex_lock( &this->count_mutex );
    //#>>>>TODO:COPY THE REQUEST MSG TO A LOCAL VARIABLE
//    float x_t = req.a;
//    float y_t = req.b;
//    float theta_t = req.c;

    this->turtlePose_desired_g(0) = req.a;
    this->turtlePose_desired_g(1) = req.b;
    this->turtlePose_desired_g(2) = req.c;

    pthread_mutex_unlock( &this->count_mutex );

    //#>>>>TODO:PLOT THE OBTAINED DATA
    ROS_INFO_STREAM("======GOTO   " <<  this->turtlePose_desired_g(0) <<"   "<< this->turtlePose_desired_g(1) <<"   "<< this->turtlePose_desired_g(2) << "=======");
    //std::cout<< this->turtlePose_desired_g(0) <<"   "<< this->turtlePose_desired_g(1) <<"   "<< this->turtlePose_desired_g(2) <<std::endl;

    res.reply=1;
    return true;
}

Vector3d TurtleClass::getLocalPose()
{
    Vector3d local;
    pthread_mutex_lock( &this->count_mutex );
    local=this->turtlePose_g;
    pthread_mutex_unlock( &this->count_mutex );

    return local;
}

Vector3d TurtleClass::getLocalDesiredPose()
{
    Vector3d local;
    pthread_mutex_lock( &this->count_mutex );
    local=this->turtlePose_desired_g;
    pthread_mutex_unlock( &this->count_mutex );

    return local;
}



void TurtleClass::getTiagoPose(const nav_msgs::Odometry::ConstPtr &msg)
{
    pthread_mutex_lock( &this->count_mutex );

    this->turtlePose_g(0)=msg->pose.pose.position.x;
    this->turtlePose_g(1)=msg->pose.pose.position.y;

    Quaterniond q;
    q.x() = msg->pose.pose.orientation.x;
    q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z;
    q.w() = msg->pose.pose.orientation.w;

    Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

    this->turtlePose_g(2) = euler(2);

    pthread_mutex_unlock( &this->count_mutex );

    //ROS_INFO_STREAM("Tiago Pose: "<<this->turtlePose_g.transpose());
}


}
