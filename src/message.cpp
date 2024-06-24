#include <cstdio>
#include "message.hpp"

Message::Message(Robot& sender)
    : msg(sender.getID()) // Will have more robot info once set up
{
}

void Message::updateWorldMessageTracker(World& world, int receiverID) {
    //std::lock_guard<std::mutex> lock(world.getWorldMutex());
    std::unordered_map<int,std::vector<Msg>>& world_msg_tracker = world.getMessageTracker();
    //std::cout << "in msg update world tracker, world msg tracker b4: " << world_msg_tracker.size() << std::endl;
    world_msg_tracker[receiverID].push_back(msg);
    //std::cout << "in msg update world tracker, world msg tracker after: " << world_msg_tracker.size() << std::endl;
}

void Message::broadcastMessage(World& world) {
    //std::cout << "IN BROADCAST" << std::endl;
    //std::lock_guard<std::mutex> lock(world.getWorldMutex()); // Commenting out because redundant with mutex at start of world functions
    //std::cout << "after mutex" << std::endl;
    
    std::unordered_map<int, Robot*> world_robot_tracker = world.getRobotTracker();
    //std::cout << "world_robot_tracker size: " << world_robot_tracker.size() << std::endl;
    
    for (const auto& pair : world_robot_tracker) {
        Robot* receiver = pair.second;
        //std::cout << "in loop" << std::endl;
        //start here and below emily - you got this :)
        if (receiver->getID() != msg.id) { // msg.id is the sender robot's ID
            //std::cout << "got receiver ID that is not sender ID duh" << std::endl;
            //std::cout << msg.id << " is sender ID" << std::endl;
            int receiverID = receiver->getID();
            //std::cout << receiverID << " is receiverID" << std::endl;
            bool inComms = world.inComms(msg.id, receiverID);
            //std::cout << "inComms check for receiverID: " << receiverID << " result: " << inComms << std::endl;
            //std::cout << "inComms " << inComms << std::endl;
            if (inComms) {
                // Message is within range, so send it to given robot via world
                //std::cout << "Message is in range, updating world message tracker for receiverID: " << receiverID << std::endl;
                updateWorldMessageTracker(world, receiverID);
                //std::cout << "Message tracker updated for receiverID: " << receiverID << std::endl;
            }
        }
    }
    //std::cout << "Finished broadcasting message" << std::endl;
}