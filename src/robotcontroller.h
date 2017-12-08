#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include <QByteArray>
#include <QObject>
#include <QTimer>
#include "robotpackets.h"
#include "robot.h"

//Forward declaration of UDPClient
class UDPClient;

/*
 * Class to handle all robots movements (THE CORE CLASS)
 * It connects robot's movement interface and packet sending
 *
 * The main idea of this class is to change packet and start or stop client when needed
 */
class RobotController : public QObject
{
    Q_OBJECT


public: Robot *robot;

    //The Client to connect with robot
public: UDPClient *client;

    QThread *clientThread;

    //Signal emited when Client is connected to robot
Q_SIGNALS:
    void connectedToRobot();

    //method to disconnect Client
    void disconnectClient();
    void connectClient();

public:
    /*
     * The packet client sends to robot
     * It is static, so no new instances are created.
     * Client, each time Timers signal is emited, gets the packet and sends it to robot
     */
    RemoteControlPacket *packet;

    RobotController(Robot *robot);
    ~RobotController();

    //This method turns light(sends once the packet
    void turnLight();
    void invokeF();

    /*
     * Fabric method which returns new instance of packet
     * NOTE: it returns a pointer, so don't forget to delete it
     */
    RemoteControlPacket * getBasicPacket();

    /*
     * Methods to operate the robot
     * Speed can be from -32000 to 32000
     * see robot headers for more info
     */
    void movePlatformDirect(int speed);
    void movePlatformRotate(int speed);
    void setFlippersUp();
    void setFlippersDown();
    void gripper(bool open);
    void elbowNeck(int speed);
    void neck(int speed);
    void waist(int speed);
    void waistUpDown(int speed);

    //stop methods called when PANIC button is called
    void stopElbowNeck();
    void stopNeck();
    void stopWaist();
    void stopWaistUpDown();
    void stopPlatformD();
    void stopPlatformR();
    void stopGripper();
    void stopFlippers();



};

#endif // ROBOTCONTROLLER_H
