#include <QObject>
#include <QThread>
#include <QCoreApplication>

#include "../headers/robotpositioncontroller.h"
#include "../headers/robot.h"
#include "../headers/robotcontroller.h"
#include "../headers/robotpackets.h"
namespace QAppPriv
{
    static int argc = 1;
    static char * argv[] = {"sharedlib.app", NULL};
    static QCoreApplication * pApp = NULL;
    static QThread * pThread = NULL;
}
/**
 * @brief Robot::Robot
 * Main object to control robot
 */
Robot::Robot():QObject()
{
    if (QAppPriv::pThread == NULL)
    {
        // Separate thread for application thread
        QAppPriv::pThread = new QThread();
        // Direct connection is mandatory
        connect(QAppPriv::pThread, SIGNAL(started()), this, SLOT(onExec()), Qt::DirectConnection);
        QAppPriv::pThread->start();
    }
    //controller based on speed values
    controller = new RobotController(this);

    //robot's speed configurations
    //can be added in Settings window
    configuration = new RobotConfiguration(12000);

    //controller based on position values
    positionController = new RobotPositionController(this);

    //handle client's got telemtry from robot
    connect(this, SIGNAL(telemetryChanged(char*)),positionController, SLOT(handleTelemetry(char*)));

}

//move robot's platform in direct way
//so both servo motors are moved in the same direction
void Robot::moveD(int speed){
    //minus because forward is <(lt)0, back is >(gt)0
    controller->movePlatformDirect(-speed);
}

//move robot's platform in rotational way
//robot's motors start to move in different directions
//can be used together with moveD() method to let the robot move in circle
void Robot::moveR(int speed){
    controller->movePlatformRotate(speed);
}

//turn robot's light
//not work after first attempt
void Robot::turnLight(){
    controller->turnLight();
}

//move robot's flippers, if direction -1, then down, 0-stop, 1-up
void Robot::flippers(int direction){
    switch (direction){
    case 1: controller->setFlippersUp(); break;
    case -1: controller->setFlippersDown(); break;
    default: controller->stopFlippers(); break;
    }
}

//opens robot's gripper(actually they go back)
void Robot::openGripper(){
    controller->gripper(true);
}

//closes gripper, so it can hold objects
void Robot::closeGripper(){
    controller->gripper(false);
}

//return config
RobotConfiguration* Robot::getConfiguration(){
    return configuration;
}

void Robot::onExec()
{
    if (QCoreApplication::instance() == NULL)
    {
        QAppPriv::pApp = new QCoreApplication(QAppPriv::argc, QAppPriv::argv);
        QAppPriv::pApp->exec();
        if (QAppPriv::pApp)
            delete QAppPriv::pApp;
    }
}

/**
 * @brief Robot::turnElbowAndNeck
 * turns both elbow and neck joints
 * so robot's hand goes up
 * @param speed - the speed with which they are moved
 *
 * NOTE: you can't move only elbow
 */
void Robot::turnElbowAndNeck(int speed){
    controller->elbowNeck(-speed);
}

//turn only neck(head joint)
//NOTE: dangerous move
void Robot::turnNeck(int speed){
    controller->neck(-speed);
}

/**
 * @brief Robot::turnWaist
 * turns robot's waist left and right
 * NOTE: you can't move platform and waist because of collision of AXIS and BUTTON values
 * @param speed
 */
void Robot::turnWaist(int speed){
    controller->waist(speed);
}

/**
 * @brief Robot::moveWaist
 * here waist is the same as shoulder
 * it moves hand up or down
 * NOTE: this may be the most dangerous movement
 * @param speed
 */
void Robot::moveWaist(int speed){
    controller->waistUpDown(speed);
}

//opens or closes gripper
void Robot::gripper(int direction){
    switch (direction) {
    case 1: controller->gripper(true);

        break;
    case -1: controller->gripper(false); break;
    default: controller->stopGripper();
        break;
    }
}

//stops all motors
//used for PANIC button
void Robot::stopAll(){
    controller->stopElbowNeck();
    controller->stopNeck();
    controller->stopWaist();
    controller->stopWaistUpDown();
    controller->stopPlatformD();
    controller->stopPlatformR();
    controller->stopFlippers();
    controller->stopGripper();
}

Robot::~Robot(){

    delete controller;
    delete configuration;
    delete positionController;
}

//set client to connect to robot
//see RobotController class for details
void Robot::connectToEngineer(){
    this->isConnecting = true;
    Q_EMIT controller->connectClient();;
}

void Robot::disconnectFromEngineer()
{
    this->isConnecting = false;
    this->isConnected = false;
    Q_EMIT controller->disconnectClient();
}


