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
    std::vector<int> bundle; // Sender bundle of tasks

    // add more robot/cbba info here later

    Msg(int id, int task_id, Pose2D location, std::vector<int> bundle)
        : id(id), task_id(task_id), location(location), bundle(bundle) {}
};

class Message {
public:
    Message(Robot& sender);

    //void printMessage(Msg msg);
    void broadcastMessage(World& world);
    void updateWorldMessageTracker(World& world, int receiverID);
    void ping(World& world);
    void updateWorldPingTracker(World& world, int receiverID, int senderID);
    //void updateRobotMessageQueue(Robot& receiver);
    //void receiveMessages(World& world, Robot& receiver);

private:
    Robot& sender;
    Msg msg; // Private member variable
  
};

#endif