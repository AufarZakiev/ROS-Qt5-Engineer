#ifndef ROBOT_H
#define ROBOT_H
#include "robotconfiguration.h"
#include <QObject>
//forward declaration of RobotController class
class RobotController;

class RobotPositionController;
//forward declaration of TelemetryPacket class
struct TelemetryPacket;


/*
 * this class works with robotController to change packets
 * needed to abstract the changing packet action
 */
class Robot : public QObject
{
    Q_OBJECT
public:

    //the object to change packets, also creates Client, see description in robotcontroller.h
    RobotController *controller;
    ~Robot();
    Robot();
    /*
     * moves robot platform directly(back or forward)
     * if speed is greater that zero, robot moves back, else
     * moves forward
     */
    void moveD(int speed);

    /*
     * rotates robot base.
     * This means that left motor turns in one side, while
     * right turns another.
     * NOTE: can be used with moveD() simultaniously to make robot move in circle
     *
     * if speed is greater than zero, robot moves right direction, else moves left
     */
    void moveR(int speed);

    //turns on or off light, note the problem that robot needs sometimes more than 1 packet
    void turnLight();

    /*
     * method moves robot's hand left or right
     * NOTE: this movement is very dangerous since robot can fall due to it's head weight
     * if speed is gt(>)0, then rotate right, else left
     */
    void turnWaist(int speed);

    /*
     * SHOULDER==MOVE WAIST
     * this joint is placed on robot's base(hand)
     * if gt(>)0, then up, else down
     * NOTE: this movement is the most dangerous! Be careful while using!
     */
    void moveWaist(int speed);

    /*
     * method moves Neck(the robot's head) up and down
     * if gt(>)0, back, else up
     * back means head moves the side with cameras up
     */
    void turnNeck(int speed);

    /*
     * robot's head moves up with this method
     * The main idea is that both neck and elbow joints move
     *
     */
    void turnElbowAndNeck(int speed);

    /*
     * method to move flippers, up means the following:
     * Imagine robot in his standart position(when all joints are down)
     * flippers are also near the base. So up moves the flippers Up so that they
     * can make robots position stable. Down means backwards.
     * NOTE: be aware of the waist and gropper position, because no safity limitations are implemented
     */
    void flippers(int direction);

    /*
     * Opens robot's gripper. Open means that the gripper can't hold something
     */
    void openGripper();

    /*
     * closes gripper so it can hold something
     */
    void closeGripper();

    /*
     * the same as the previous methods
     */
    void gripper(int direction);


    /*stop methods */
    void stopGripper();
    void stopFlippers();
    void stopAll();

    /*
     * method called when you need to connect to Robot
     * Actually it calles robotController's method, which envokes UDPClient
     */
    void connectToEngineer();
    void disconnectFromEngineer();

    //returns configuration object
    RobotConfiguration* getConfiguration();


Q_SIGNALS:
    /*
     * this signal is emited when UdpClient gets new packet from robot.
     * The slot from MainWindow handles it and shows the info
     */
    void telemetryChanged(char *data);
    void videoFrameSended(char *data, int length);

public Q_SLOTS:
    void onExec();

public:
    //Flag to handle connection
    bool isConnected = false;
    bool isConnecting = false;
    RobotConfiguration *configuration;
    RobotPositionController *positionController;
};

#endif // ROBOT_H
