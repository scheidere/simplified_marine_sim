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
    std::unordered_map<int,int> winners; // CBBA only
    std::unordered_map<int,double> winning_bids; // CBBA only
    std::vector<std::vector<double>> winning_bids_matrix; // CBGA only
    std::unordered_map<int,double> timestamps; // When sender last received message from each neighboring agent (no key for self ID, no message sending to self)
    std::unordered_map<int,Pose2D> locations; // Added for CBGA! For flexibility, using Pose2D x and y to denote location, ignoring theta. Includes key for self location value.

    Msg(int id, std::unordered_map<int,int> winners, std::unordered_map<int,double> winning_bids, std::vector<std::vector<double>> winning_bids_matrix, std::unordered_map<int,double> timestamps, std::unordered_map<int,Pose2D> locations)
        : id(id), winners(winners), winning_bids(winning_bids), winning_bids_matrix(winning_bids_matrix), timestamps(timestamps), locations(locations) {} 

};

class Message {
public:
    Message(Robot& sender);

    //void printMessage(Msg msg);
    void broadcastMessage(World& world);
    void updateWorldMessageTracker(World& world, int receiverID);
    void ping(World& world);
    void updateWorldPingTracker(World& world, int receiverID, int senderID, double senderTimestamp);
    //void updateRobotMessageQueue(Robot& receiver);
    //void receiveMessages(World& world, Robot& receiver);

    //double getTimestamp();

private:
    Robot& sender;
    Msg msg; // Private member variable

  
};

#endif