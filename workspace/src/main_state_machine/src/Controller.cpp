#include <iostream>
#include <cstdlib>
#include "Controller.h"

Controller::Controller(QObject * parent) :QObject(parent),referee(0){

    // Instantiate a referee
    referee = new Referee(3, this);

    // Connect this->slots with Angelina's signals
    connect(referee, SIGNAL(disconnected()), this, SLOT(slotDisconnected()));
    connect(referee, SIGNAL(detectionStart()), this, SLOT(slotDetectionStart()));
    connect(referee, SIGNAL(gameStart()), this, SLOT(slotGameStart()));
    connect(referee, SIGNAL(trueColorOfTeam(TeamColor)), this, SLOT(slotTrueColorOfTeam(TeamColor)));
    connect(referee, SIGNAL(gameOver()), this, SLOT(slotGameOver()));
    connect(referee, SIGNAL(stopMovement()), this, SLOT(slotStopMovement()));
    connect(referee, SIGNAL(connected()), this, SLOT(slotConnected()));
    connect(referee, SIGNAL(connectFailed()), this, SLOT(slotConnectFailed()));
    connect(referee, SIGNAL(abValues(double,double)), this, SLOT(slotAbValues(double,double)));

    // Initiate flags for main state machine
    isConnected = false;
    need_reconnect = false;
    wrong_color = false;
    ab_arrived = false;
    de_started = false;
    game_started = false;
    ended = false;
    paused = false;
}

Controller::~Controller() {

    delete referee;
}

// Report that we are ready, will be called by the main state machine
void Controller::reportReady(){

    referee->reportReady();

}

// A QTimer that sends an Alive signal each 15 secs
void Controller::start_alive_timer(){
    QTimer *aliveTimer = new QTimer(this);
    connect(aliveTimer, SIGNAL(timeout()), this, SLOT(slotSendAlive()));
    aliveTimer->start(15000);
}

// Receive A and B from Angelina
void Controller::slotTellAbValue(double ratio)
{

    referee->tellAbRatio(ratio);
}

// Tell Angelina the detected color
void Controller::tellTeamColor(TeamColor color)
{
    //Teamcolor colors = color;
    detected_color = color;
    referee->tellTeamColor(color);
}

// Tell Angelina the detected AB ratio
void Controller::tellABRatio(double ratio)
{
    referee->tellAbRatio(ratio);
}

// Report Angelina a goal
void Controller::reportGoal()
{
    referee->reportGoal();
}

// Report we are done
void Controller::reportDone()
{
    referee->reportDone();
}

// Connect to Angelina
void Controller::connectToServer(const QString &ip, int port)
{
    std::cout << "Connecting to server..." << std::endl;
    referee->connectToServer(ip, port);
}

// SendAlive connected with a QTimer
void Controller::slotSendAlive()
{
      std::cout << "send alive, send alive" << std::endl;
      referee->sendAlive();
}

// Wird gesendet, falls die Verbindung zum Server erfolgreich war.
void Controller::slotConnected() {

    isConnected = true;
    need_reconnect = false;

}

void Controller::slotConnectFailed() {

    std::cout << ("Connection failed.") << std::endl;
    isConnected = false;
    need_reconnect = true;
}

void Controller::slotDisconnected() {

    paused = true;
}

// Receive signal for starting playground detection
void Controller::slotDetectionStart() {

    de_started = true;
    game_started = false;
    paused = false;
    //changeStatus(status_free_explore);

}

// Receive game started signal
void Controller::slotGameStart() {

     game_started = true;
     de_started = false;
     paused = false;
    //changeStatus(status_select_target);

}

// Receive the true team color
void Controller::slotTrueColorOfTeam(TeamColor color) {

    if (color == blue) {

        US_COLOR = blue;
        OP_COLOR = yellow;

        std::cout << "We are now blue" << std::endl;
    } else {

        US_COLOR = yellow;
        OP_COLOR = blue;

        std::cout << "We are yellow" << std::endl;
    }
    if (detected_color != color) {
        wrong_color = true;
    }

}

// Receive true A and B values
void Controller::slotAbValues(double a, double b)
{
    ab_arrived = true;
    true_a = a;
    true_b = b;
}

// Receive gameover signal
void Controller::slotGameOver() {
    ended = true;
}

// Receive pause signal
void Controller::slotStopMovement() {
    paused = true;
}



