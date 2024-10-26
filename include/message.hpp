#ifndef MESSAGE_H
#define MESSAGE_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "world.hpp"
#include "robot.hpp"
//#include "CBBA.hpp"
#include "structs.hpp"

struct Msg {
    int id; // sender robot ID
    int task_id; // Task sender is currently doing
    Pose2D location; // Current location of sender
    Bundle bundle; // Sender bundle of tasks

    Msg(int id, int task_id, Pose2D location, Bundle bundle)
        : id(id), task_id(task_id), location(location), bundle(bundle) {}
};

class Message {
public:
    Message(Robot& sender);

    //void printMessage(Msg msg);
    void broadcastMessage(World& world);
    void updateWorldMessageTracker(World& world, int receiverID);
    //void updateRobotMessageQueue(Robot& receiver);
    //void receiveMessages(World& world, Robot& receiver);

private:
    Msg msg; // Private member variable
    
};

#endif