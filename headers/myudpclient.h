#ifndef MYUDPCLIENT_H
#define MYUDPCLIENT_H
#include <QObject>
#include <QtNetwork/QUdpSocket>
#include <QThread>
#include <QTimer>
#include <QDataStream>
#include <QHostAddress>
#include "robotpackets.h"
#include "robot.h"

//standart port to work with robot, see Servosila documentation for more details
#define ROBOT_PORT 10000

//forward decalration of RobotController class
class RobotController;

class UDPClient : public QObject{
    Q_OBJECT

    //my slots
public slots:
    /*
     * method starts timer to send packets to robot
     * uses signal of QTimer, gets packet from RobotController
     */
    void startTimerTask();

    /*
         * sends packets each time called, needed to connect with startTimerTask();
         */
    void sendLivePackets();

    /*
         * slot connected to socket to be called, when telemetry(or other) packet received
         * from robot. Emits ui slot (see MainWindow) to show them
         */
    void listenRobot();
    void connectToRobot();
    void disconnectFromRobot();



private:
    //handles the connection state (Flag)
    bool isConntected = false;
    //socket to work with Robot, UDP
    QUdpSocket *m_pudp;
    QHostAddress *robotAddress;
    void writeInputToFile(char *data);


    //controller needed to get packets
    RobotController *controller;
    Robot* robot;

    //needed to send live packets specific time
    QTimer *timer;


public:
    //called when Disconnect button called


    UDPClient(RobotController *controller);

    //sends single packet, needed for Light
    void sendPacket(RemoteControlPacket packet);
    void moveToThread(QThread *t);
    ~UDPClient();

    //method that starts to connect to robot, entry method(!)
};
#endif // UDPCLIENT_H
