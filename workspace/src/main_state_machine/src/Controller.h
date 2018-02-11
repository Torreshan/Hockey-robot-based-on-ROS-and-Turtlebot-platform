#include <QTimer>
#include <QWidget>
#include <QObject>
#include <QMap>

#include "referee.h"

/**
 * @brief The Controller class implements communication funtionalities with angelina. In this class, a
 * referee is instantiated. -- Class written and maintained by Fengze Han and Daoping Wang
 */
class Controller:public QObject
{
    Q_OBJECT

public:
    Controller(QObject *parent = 0);
    ~Controller();

    // Hermes referee instance
    Referee *referee;
    
    // Color of our team, told by Angelina
    TeamColor US_COLOR;

    // Color of the enemy
    TeamColor OP_COLOR;

    // Detected team color
    TeamColor detected_color;

    // Flags for the main state machine for controlling purpose
    bool need_reconnect;
    bool isConnected;
    bool de_started;
    bool game_started;
    bool ended;
    bool wrong_color;
    bool ab_arrived;
    bool paused;

    // The true A and B told by Angelina
    double true_a;
    double true_b;

    // Functions for calling referee
    void reportReady();
    void changeStatus();
    void tellTeamColor(TeamColor color);
    void tellABRatio(double ratio);
    void reportGoal();
    void reportDone();
    void connectToServer(const QString &ip, int port);
    void start_alive_timer();

    // Slots for Angelina signals
public Q_SLOTS:
    void slotTellAbValue(double ratio);
    void slotSendAlive();
    void slotConnected();
    void slotConnectFailed();
    void slotDisconnected();
    void slotDetectionStart();
    void slotGameStart();
    void slotTrueColorOfTeam(TeamColor);
    void slotAbValues(double a, double b);
    void slotGameOver();
    void slotStopMovement();
};
