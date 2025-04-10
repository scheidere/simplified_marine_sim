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
    //int task_id; // Task sender is currently doing
    //Pose2D location; // Current location of sender
    std::unordered_map<int,int> winners;
    std::unordered_map<int,double> winning_bids;
    std::unordered_map<int,double> timestamps;
    // double timestamp;

    // add more robot/cbba info here later

    Msg(int id, std::unordered_map<int,int> winners, std::unordered_map<int,double> winning_bids, std::unordered_map<int,double> timestamps)
        : id(id), winners(winners), winning_bids(winning_bids), timestamps(timestamps) {} 

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

    //double getTimestamp();

private:
    Robot& sender;
    Msg msg; // Private member variable

  
};

#endif