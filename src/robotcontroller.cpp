#include <QObject>
#include <cfloat>
#include <algorithm>

#include "../headers/robotcontroller.h"
#include "../headers/myudpclient.h"
#include "../headers/robotpackets.h"
#include "../headers/robot.h"

/**
 * @brief RobotController::RobotController
 * This class controls robot's movement, based on speed values
 *
 * @param r - the pointer to Robot class object
 */
RobotController::RobotController(Robot *r):QObject()
{
    robot = r;

    //client to handle
    client = new UDPClient(this);
    //its thread
    clientThread = new QThread();
    //build packet, which is static
    //client will use it to send, and all other methods change it
    packet = getBasicPacket();
    //move client to another thread
    client->moveToThread(clientThread);

    //connect it to make it parallel
//    connect(clientThread,SIGNAL(started()), client,SLOT(connectToRobot()));
    connect(this,SIGNAL(connectClient()), client,SLOT(connectToRobot()));
    connect(this, SIGNAL(disconnectClient()), client, SLOT(disconnectFromRobot()));
    clientThread->start();

}

RobotController::~RobotController(){
    clientThread->quit();
    if(!clientThread->wait(3000)) //Wait until it actually has terminated (max. 3 sec)
    {
        clientThread->terminate(); //Thread didn't exit in time, probably deadlocked, terminate it!
        clientThread->wait(); //We have to wait again here!
    }
    delete client;
    delete clientThread;
    delete packet;
}

/**
 * @brief RobotController::turnLight
 * turns light, main problem is that there is no info about the light
 */
void RobotController::turnLight(){
    RemoteControlPacket *packet = getBasicPacket();
    packet->BUTTON[1] = 1;
    //    client->sendPacket(*packet);
}


//returns packet with zero values
RemoteControlPacket* RobotController::getBasicPacket(){

    RemoteControlPacket *packet = new RemoteControlPacket();
    packet->AXIS[16] = {0};
    packet->BUTTON[16] = {0};
    packet->TELEMETRY = 0.0;
    return packet;
}

/**
  * METHODS BELOW IMPLEMENT SERVOSILA'S DOCUMENTATION
  * SEE IT FOR MORE DETAILS
  * START SECTION
  * ==============================================================================
  */
/*
 * sets the AXIS[1] speed to @speed, this moves platform
 */
void RobotController::movePlatformDirect(int speed){
    packet->AXIS[1] = speed;
}



void RobotController::movePlatformRotate(int speed){
    packet->AXIS[0] = speed;
}

void RobotController::setFlippersUp(){
    packet->AXIS[5] = 1;

}


void RobotController::setFlippersDown(){
    packet->BUTTON[5] = 1;
}

/*
 * if open = true, then open gripper,
 * else close
 */
void RobotController::gripper(bool open){
    if(open){
        packet->BUTTON[0] = 1;
    }
    else packet->BUTTON[3] = 1;
}

void RobotController::neck(int speed){
    packet->AXIS[4] = speed;
    packet->BUTTON[10] = 1;
}
void RobotController::elbowNeck(int speed){
    packet->AXIS[4] = speed;

}

void RobotController::waist(int speed){
    packet->BUTTON[9] = 1;
    packet->AXIS[0] = speed;
}

void RobotController::waistUpDown(int speed){
    packet->BUTTON[9] = 1;
    packet->AXIS[1] = speed;
}


void RobotController::stopWaist(){
    packet->BUTTON[9] = 0;
    packet->AXIS[0] = 0;
}
void RobotController::stopWaistUpDown(){
    packet->BUTTON[9] = 0;
    packet->AXIS[1] = 0;
}
void RobotController::stopPlatformD(){
    packet->AXIS[1] = 0;
}
void RobotController::stopPlatformR(){
    packet->AXIS[0] = 0;
}

void RobotController::stopElbowNeck(){
    packet->AXIS[4] = 0;
    packet->BUTTON[10] = 0;
}

void RobotController::stopNeck(){
    packet->AXIS[4] = 0;
}

void RobotController::stopFlippers(){
    packet->BUTTON[5] = 0;
    packet->AXIS[5] = 0;
}
void RobotController::stopGripper(){
    packet->BUTTON[0] = 0;
    packet->BUTTON[3] = 0;
}

/**
 * END SECTION
 * =============================================================================
 */

//start only clientThread, which will call
//client's method via Signal-Slot connection
//void RobotController::connectClient(){
//    clientThread->start();
//}

//disconnect
//void RobotController::disconnectClient(){
//    client->disconnectFromRobot();
//}
