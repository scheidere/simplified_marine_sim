#include <cstdio>
#include "message.hpp"

Message::Message(Robot& sender)
    : sender(sender),
      msg(sender.getID(), sender.getWinners(), sender.getWinningBids(), sender.getTimestamps()) // Timestamp updated upon receipt by another robot
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

        // TESTING ONLY
        /*if (pair.first == msg.id) {
            // Found sender
            Robot* sender = pair.second;
            sender->log_info("in broadcastMessage!!!!");
        }
*/

        //start here and below emily - you got this :)
        if (receiver->getID() != msg.id) { // msg.id is the sender robot's ID
            //std::cout << "got receiver ID that is not sender ID duh" << std::endl;
            //std::cout << msg.id << " is sender ID" << std::endl;
            int receiverID = receiver->getID();
            //receiver->log_info("in broadcastMessage but receiving part where world message tracker should be updated");
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


    // Adding a delay to help with concurrency issue if nothing is done between sending and listening for messages (in BT structure)
    

    //std::cout << "Finished broadcasting message" << std::endl;
}

void Message::updateWorldPingTracker(World& world, int receiverID, int senderID) {
    //std::lock_guard<std::mutex> lock(world.getWorldMutex());
    std::unordered_map<int,std::vector<int>>& world_ping_tracker = world.getPingTracker();
    //std::cout << "in msg update world tracker, world msg tracker b4: " << world_msg_tracker.size() << std::endl;

    if (receiverID == senderID) { 
        std::cerr << "ERROR: Robot " << senderID << " is adding itself to ping tracker! Fix this!" << std::endl;
        return; 
    }

    world_ping_tracker[receiverID].push_back(senderID);
    std::string bla = "Ping to robot " + std::to_string(receiverID) + " from robot " + std::to_string(senderID);
    sender.log_info(bla);
    std::cout << "in ping update world tracker, world ping tracker after: " << world_ping_tracker.size() << std::endl;
}

void Message::ping(World& world) {

    std::unordered_map<int, Robot*> world_robot_tracker = world.getRobotTracker();
    
    int senderID = sender.getID();
    sender.log_info("In ping");

    for (const auto& pair : world_robot_tracker) {
        Robot* receiver = pair.second;
        int receiverID = receiver->getID();

        if (receiverID != senderID) { // as long as not self

            bool inComms = world.inComms(senderID, receiverID);
            if (inComms) {
                //std::string bla = "DEBUG: Robot " + std::to_string(senderID) + " pinging Robot " + std::to_string(receiverID);
                //sender.log_info(bla);
                // In range, so update tracker so each receiver will know what robots are newly in range (but haven't send messages yet)
                updateWorldPingTracker(world, receiverID, senderID);
            }
        }
    }
}

/*double Message::getTimestamp() { //do we want this here
    // Time of last information update from other robot
}
*/

/*void Message::printMessage(Msg msg) {
    std::cout << "Message ID:" << msg.id << ":\n";
    std::cout << "Task ID: " << msg.task_id << "\n";
    std::cout << "Location: (" << msg.location.x << ", " << msg.location.y << ", " << msg.location.theta << ")\n";
   /* std::cout << "Bundle: [";
    for (const auto& task : msg.bundle.tasks) {
        std::cout << task << " "; // Assuming tasks can be printed this way
    }
    std::cout << "]\n";
}*/