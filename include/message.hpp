#ifndef MESSAGE_H
#define MESSAGE_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "robot.hpp"
#include "world.hpp"

struct Msg {
    int id;
    // Add more later, like pose

    Msg(int id)
        : id(id) {}
};

class Message {
public:
    Message(Robot& sender);

    void broadcastMessage(World& world);
    void updateWorldMessageTracker(World& world, int receiverID);
    //void updateRobotMessageQueue(Robot& receiver);
    //void receiveMessages(World& world, Robot& receiver);

private:
    Msg msg; // Private member variable
    
};

/*class Message {
protected:
    int robot_id; // ID of robot sending message


public:
    Message(int robot_id);

    int getSenderID() const { return robot_id; }

    void sendMessage(int origin_robot_id);

};
*/
#endif