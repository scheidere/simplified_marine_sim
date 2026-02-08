#include <cstdio>
#include "message.hpp"
#include "utils.hpp"


Message::Message(Robot& sender)
    : sender(sender),
      msg(sender.getID(), sender.getWinners(), sender.getWinningBids(), sender.getWinningBidsMatrix(), sender.getTimestamps(), sender.getLocations(), sender.getTaskProgress(), sender.getSubtaskFailures(), sender.getDiscoveredObstacles()) // Timestamp updated upon receipt by another robot
{
    sender.log_info("CREATING Message object, reading subtask_failures now");
}

void Message::updateWorldMessageTracker(World& world, int receiverID) {
    std::lock_guard<std::mutex> lock(world.getWorldMutex());
    std::unordered_map<int,std::vector<Msg>>& world_msg_tracker = world.getMessageTrackerUnsafe(); // this does work better, allows helpful mutex above to protect pushback below
    
    //std::unordered_map<int,std::vector<Msg>>& world_msg_tracker = world.getMessageTracker(); // must not have mutex above to use this

    //std::cout << "in msg update world tracker, world msg tracker b4: " << world_msg_tracker.size() << std::endl;
    world_msg_tracker[receiverID].push_back(msg);
    //std::cout << "in msg update world tracker, world msg tracker after: " << world_msg_tracker.size() << std::endl;
}

void Message::broadcastMessage(World& world) {
    //std::cout << "IN BROADCAST" << std::endl;
    //std::lock_guard<std::mutex> lock(world.getWorldMutex()); // Commenting out because redundant with mutex at start of world functions
    //std::cout << "after mutex" << std::endl;

    // std::string log_msg = "Robot " + std::to_string(msg.id) + " starting broadcast";
    // robot.log_info(log_msg); // dont have access to inividual robot here
    
    //std::unordered_map<int, Robot*> world_robot_tracker = world.getRobotTracker();
    std::map<int, Robot*>& world_robot_tracker = world.getRobotTracker();
    //std::cout << "world_robot_tracker size: " << world_robot_tracker.size() << std::endl;

    // testing
    // std::string order_msg = "Robot iteration order: ";
    // for (const auto& pair : world_robot_tracker) {
    //     order_msg += std::to_string(pair.first) + " ";
    // }
    // world.log_info(order_msg);
    
    for (const auto& pair : world_robot_tracker) {
        Robot* receiver = pair.second;
        //std::cout << "in loop" << std::endl;

        //std::string plz = "Checking for message to robot id: " + std::to_string(pair.first); 
        if (pair.first != msg.id) { // only print if from actual sender, excluding receiver id pair.first
            std::string plz = "Robot " + std::to_string(msg.id) + " checking broadcast to robot " + std::to_string(pair.first);
            receiver->log_info(plz);
        }

        // TESTING ONLY
        // if (pair.first == msg.id) {
        //     // Found sender
        //     Robot* sender = pair.second;
        //     sender->log_info("in broadcastMessage!!!!");
        //     sender->log_info("Winners: ");
        //     utils::logUnorderedMap(msg.winners,*sender);
        //     sender->log_info("Winning bids: ");
        //     utils::logUnorderedMap(msg.winning_bids,*sender);
        //     sender->log_info("Timestamps: ");
        //     utils::logUnorderedMap(msg.timestamps,*sender);
        // }


        if (receiver->getID() != msg.id) { // msg.id is the sender robot's ID
            //std::cout << "got receiver ID that is not sender ID duh" << std::endl;
            //std::cout << msg.id << " is sender ID" << std::endl;
            int receiverID = receiver->getID();
            //receiver->log_info("in broadcastMessage but receiving part where world message tracker should be updated");
            //std::cout << receiverID << " is receiverID" << std::endl;
            bool inComms = world.inComms(msg.id, receiverID);
            //std::cout << "inComms check for receiverID: " << receiverID << " result: " << inComms << std::endl;
            //std::cout << "inComms " << inComms << std::endl;

            receiver->log_info("Robot " + std::to_string(msg.id) + " -> Robot " + std::to_string(receiverID) + ": inComms = " + (inComms ? "true" : "false")); // for testing

            if (inComms) {
                // Message is within range, so send it to given robot via world
                std::string log_msg = "Adding message: Robot " + std::to_string(msg.id) + " -> Robot " + std::to_string(receiverID);
                receiver->log_info(log_msg);
                //std::cout << "Message is in range, updating world message tracker for receiverID: " << receiverID << std::endl;
                //world.log_info("ATTEMPTING STORAGE: Robot " + std::to_string(msg.id) + " -> Robot " + std::to_string(receiverID));
                updateWorldMessageTracker(world, receiverID);
                // world.log_info("STORAGE COMPLETE: Robot " + std::to_string(msg.id) + " -> Robot " + std::to_string(receiverID));
                std::string log_msg2 = "Message added successfully";
                receiver->log_info(log_msg2);

                //std::cout << "Message tracker updated for receiverID: " << receiverID << std::endl;
            }
        }
    }


    // Adding a delay to help with concurrency issue if nothing is done between sending and listening for messages (in BT structure)
    

    //std::cout << "Finished broadcasting message" << std::endl;
    std::string log_msg2 = "Robot " + std::to_string(msg.id) + " finished broadcast";

}

void Message::updateWorldPingTracker(World& world, int receiverID, int senderID, double senderTimestamp, bool senderSubtaskFailureFlag) {
    //std::lock_guard<std::mutex> lock(world.getWorldMutex());
    std::unordered_map<int,std::vector<std::tuple<int,double,bool>>>& world_ping_tracker = world.getPingTracker();
    //std::cout << "in msg update world tracker, world msg tracker b4: " << world_msg_tracker.size() << std::endl;

    sender.log_info("in updateWorldPingTracker");
    std::string hay = "senderSubtaskFailureFlag in ping: " + std::to_string(senderSubtaskFailureFlag);
    sender.log_info(hay);

    if (receiverID == senderID) { 
        std::cerr << "ERROR: Robot " << senderID << " is adding itself to ping tracker! Fix this!" << std::endl;
        return;
    }

    sender.log_info("here");

    // Search for senderID in the pings
    auto& receiver_pings = world_ping_tracker[receiverID];
    auto it = std::find_if(receiver_pings.begin(), receiver_pings.end(), 
                          [senderID](const std::tuple<int,double,bool>& p) { 
                              auto [sender_id, timestamp, flag] = p;
                              return sender_id == senderID; 
                          });
    
    // If didn't find ping from sender already
    if (it == receiver_pings.end()) {
        std::string jfc = "senderSubtaskFailureFlag 3: " + std::to_string(senderSubtaskFailureFlag);
        sender.log_info(jfc);
        receiver_pings.push_back({senderID, senderTimestamp, senderSubtaskFailureFlag}); // Add ping
    } else { // Ping from robot already heard so just update timestamp and failure flag
        std::string jfc = "senderSubtaskFailureFlag 4: " + std::to_string(senderSubtaskFailureFlag);
        sender.log_info(jfc);
        *it = {senderID, senderTimestamp, senderSubtaskFailureFlag};
    }

    if (senderSubtaskFailureFlag) { // for debugging
        world.log_info("Receiver pings in world tracker after update:");
        utils::log1DVectorFromWorld(receiver_pings, world);
    }
}

void Message::ping(World& world) {

    sender.log_info("in pinggg");

    // std::unordered_map<int, Robot*> world_robot_tracker = world.getRobotTracker();
    std::map<int, Robot*> world_robot_tracker = world.getRobotTracker(); // want order to be consistent so need map
    
    int senderID = sender.getID();
    //sender.log_info("In ping");
    double senderTimestamp = sender.getCurrentTime(); // NOTE: this timestamp just represents the time a ping is broadcasted (not anything to do with last CBBA belief update now)

    bool senderSubtaskFailureFlag = sender.getNewSelfSubtaskFailureFlag();

    std::string hir = "senderSubtaskFailureFlag: " + std::to_string(senderSubtaskFailureFlag);
    sender.log_info(hir);

    for (const auto& pair : world_robot_tracker) {
        Robot* receiver = pair.second;
        int receiverID = receiver->getID();

        if (receiverID != senderID) { // as long as not self

            sender.log_info("before incomms in ping");
            bool inComms = world.inComms(senderID, receiverID);
            sender.log_info("after incomms in ping");
            if (inComms) {
                //std::string bla = "DEBUG: Robot " + std::to_string(senderID) + " pinging Robot " + std::to_string(receiverID);
                //sender.log_info(bla);
                // In range, so update tracker so each receiver will know what robots are newly in range (but haven't send messages yet)
                updateWorldPingTracker(world, receiverID, senderID, senderTimestamp, senderSubtaskFailureFlag);
            }
        }
    }
}