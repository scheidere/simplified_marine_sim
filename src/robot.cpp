#include <cstdio>
#include <stdexcept> // For exception handling
#include "planners.hpp"
#include "world.hpp"
#include "robot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/actions/pop_from_queue.hpp"
#include "parser.hpp"
#include "utils.hpp"


// Create state class, which will contain robot condition functions

// Note we init winning_bids and winning_agent_indices with numTasks of 1 because of access issues to numTasks during initialization
Robot::Robot(Planner* p, ShortestPath* sp, CoveragePath* cp, World* w, JSONParser* psr, const Pose2D& initial_pose, const Pose2D& goal_pose, int robot_id, std::string robot_type, cv::Scalar dot_color) 
: planner(p), shortest_path(sp), coverage_path(cp), world(w), parser(psr), id(robot_id), type(robot_type) {
    
    // Get filename for logging and then open it
    std::string filename = generateLogFilename();
    std::ofstream clear(filename, std::ios::out); // Clear logging file from previous run
    clear.close();
    robot_log.open(filename, std::ios::app); // Allow appending

    pose = {0, 0, 0};
    goal = goal_pose; // Like return to home or drop off item loc, specific to each robot
    color = dot_color;
    id = robot_id;
    type = robot_type;
    task_id = 0; // ID of current task, have zero represent undefined
    world->trackRobot(this);
    battery_level = 1.0;
    init(initial_pose);
    //assignable_tasks = tasks; // Tasks that can be assigned to this robot

    doable_task_ids = world->getRobotCapabilities(this); // doable local tasks (by id)
    //std::cout << "Printing capabilities..." << std::endl;
    //utils::print1DVector(capabilities);
    //std::cin.get();

    std::string hi = "doable_task_ids: ";
    log_info(hi);
    utils::log1DVector(doable_task_ids, *this);

    max_depth = parser->getMaxDepth(); // Also in CBBA

    bundle.resize(max_depth, -1); // init bundle
    path.resize(max_depth, -1);
    //scores.resize(max_depth, 0.0);

    // Bids/winners/winning_bids initialized here :)
    bids = initBids();
    log_info("Bids (task id: double): ");
    //utils::logUnorderedMap(bids, *this);
    utils::logMap(bids, *this);
    winners = initWinners();
    log_info("Winners (task id: int): ");
    utils::logUnorderedMap(winners, *this);
    winning_bids = initWinningBids();
    log_info("Winning bids (task id: double): ");
    utils::logUnorderedMap(winning_bids, *this);
    timestamps = initTimestamps();

    // Inits for tracking of previous info for convergence checks between iterations
    prev_bundle.resize(max_depth, -1);
    prev_path.resize(max_depth, -1);
    prev_winners = initWinners();
    prev_winning_bids = initWinningBids();
    num_converged_iterations = 0;

    // Can robot do task i at j idx in bundle
    //feasible_tasks = world->initFeasibleTasks(this); // dimensions: num_tasks x max_depth

    std::cout << "Robot constructor: Getting all tasks from world..." << std::endl;
    //std::vector<Task> allTasks = world->getAllTasks();
    int numTasks = world->getNumLocalTasks(); //allTasks.size(); // why are we doing this?? check it's needed
    std::cout << "Number of tasks in world: " << numTasks << std::endl;

    // Elements in bundle and path will be numeric task IDs, so 1+ (-1 if unassigned)
    // There is a check in CBBA::BuildBundle to init them to have total num (max_depth) elements with -1 initially
    //bundle = std::vector<int>(max_depth, -1); //doesn't work cuz no access to max_depth here, it's in cbba

    // Sanity check on task size
    if (numTasks > 1000000) {
        std::cerr << "Warning: Number of tasks is unusually large: " << numTasks << std::endl;
    }

    cbba_rounds = 0;
    last_pings = {}; // Initializing last tracked pings with empty vector of ints
    time_of_last_self_update = -1.0; // Just to avoid issues when it has no value

    comms_timeout_threshold = 5.0;

}

std::string Robot::generateLogFilename() {
    std::ostringstream oss;
    oss << "robot_" << id << "_log.txt";  // Create a string in the format "robot_<robotID>_log.txt"
    return oss.str();
}

void Robot::log_info(std::string log_msg) {

    if (robot_log.is_open()) {
        //std::cerr << "LOG FILE IS OPEN FOR ROBOT " << id << std::endl;
        //std::cin.get();

        //robot_log << "Testing: " << id << std::endl;
        robot_log << log_msg << std::endl;
        //robot_log.flush(); // Write immediately
        //robot_log.close();

    } else {
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Failed to open log file for Robot ID " << id << std::endl;
    }

}

//std::unordered_map<int,double> Robot::initBids() {
std::map<int,double> Robot::initBids() {

    //std::unordered_map<int,double> bids;
    std::map<int,double> bids;

    for (int task_id : doable_task_ids) { // traverse doable tasks
       bids[task_id] = 0.0; // Initialize each key with 0.0
    }

    return bids; 
}

std::unordered_map<int,int> Robot::initWinners() {

    std::unordered_map<int,int> winners;

    for (int task_id : doable_task_ids) { // traverse doable tasks
       winners[task_id] = -1; // Initialize each key with -1 to represent no winners
    }

    return winners;
}

std::unordered_map<int,double> Robot::initWinningBids() {

    std::unordered_map<int,double> winning_bids;

    for (int task_id : doable_task_ids) { // traverse doable tasks
       winning_bids[task_id] = 0.0; // Initialize each key with 0.0
    }

    return winning_bids; 

}

std::unordered_map<int,double> Robot::initTimestamps() {

    std::unordered_map<int,double> timestamps;

    // Get agentinfo from world
    std::unordered_map<int,AgentInfo> all_agents_info = world->getAllAgentsInfo();

    for (const auto& [id_k, agent] : all_agents_info) {
        // For all id's k, not equal to current id i, init timestamp
        if (id_k != id) {
            timestamps[id_k] = -1; 
        }
    }

    return timestamps; 

}

void Robot::init (Pose2D initial_pose) {
    std::cout << "Initializing robot pose..." << std::endl;
    // Access world for image
    cv::Mat image = world->getImage();
    cv::circle(image, cv::Point(initial_pose.x, initial_pose.y), 5, color, -1);
    pose = initial_pose; // Set current robot pose variable
}

void Robot::updateBatteryLevel(double drain_percent) {
    battery_level -= drain_percent * battery_level;
    if (battery_level < 0) battery_level = 0; // Cap at minimum of zero
}

bool Robot::batteryLow() {
    return battery_level < 0.15;
}

void Robot::move(Pose2D waypoint) {
    //std::cout << "In move for robot with ID " << getID() << std::endl;

    world->clear(pose); // Clear all robot dots (technically only need to clear moving ones, but this is easier)
    //std::cout << "Before: ";
    //world->printTrackedRobots();
    pose = waypoint; // Update x and y within robot class
    //std::cout << "After: ";
    //world->printTrackedRobots();
    world->plot(); // Add dots at all robot locations

    double drain_percent = 0.01;
    updateBatteryLevel(drain_percent);
    //std::cout << "New battery level after move: " << battery_level << " for robot ID " << id << std::endl;
}

void Robot::updateRobotMessageQueue(Msg msg) {

    //std::cout << "IN updateRobotMessageQueue len message_queue: " << message_queue.size() << std::endl;
    try {
        // message_queue.push_back(msg); // using only this line, we have old and new messages from same senders, only want to keep newest

        // First, we search for message already in queue with same sender id as given new msg
        auto it = std::find_if(message_queue.begin(), message_queue.end(),
                              [&msg](const Msg& existing_msg) {
                                  return existing_msg.id == msg.id;});

        // If did not find a message from the sender, just add it
        if (it == message_queue.end()) {
            message_queue.push_back(msg);
        } else {
            *it = msg; // Replace with msg because it was more recently received (we assume that these messages are received in order of broadcast)
        }


    } catch (const std::exception& e) {
        std::cerr << "Exception caught while updating message queue: " << e.what() << std::endl;
        throw;
    }
    //std::cout << "END updateRobotMessageQueue len message_queue: " << message_queue.size() << std::endl;
}

void Robot::receiveMessages() {
    //std::cout << "in receiveMessages........" << std::endl;
    std::unordered_map<int, std::vector<Msg>>& world_msg_tracker = world->getMessageTracker();

    int receiverID = getID();
    //std::cout << "receiverID: " << receiverID << std::endl;
    //std::cout << "Printed msg tracker from world: " << std::endl;
    //world->printMessageTracker();
    //std::cout << "world msg tracker printed above, size = " << world_msg_tracker.size() << std::endl;

    if (world_msg_tracker.find(receiverID) != world_msg_tracker.end()) {
        std::cout << "in if" << std::endl;
        std::vector<Msg>& messages = world_msg_tracker[receiverID];
        //std::cout << "length of messages from world (should be 1?): " << messages.size() << std::endl;
        //std::cout << "!messages.empty(): " << !messages.empty() << std::endl;

        if (!messages.empty()) {
            Msg msg = messages.front();
            messages.erase(messages.begin());  // Remove the message after reading
            //timestamps[msg.id] = getCurrentTime(); this is done in updateTimestamps
            // log_info("in receiveMessages!!!!");
            // log_info("Size of world messages queue for current robot: ");
            // log_info(std::to_string(messages.size()));
            // log_info("Winners: ");
            // utils::logUnorderedMap(msg.winners,*this);
            // log_info("Winning bids: ");
            // utils::logUnorderedMap(msg.winning_bids,*this);
            // log_info("Timestamps: ");
            // utils::logUnorderedMap(msg.timestamps,*this);
            updateRobotMessageQueue(msg);
            //std::cout << "Robot " << getID() << " received a message from Robot " << msg.id << std::endl;
            std::string log_msg = "Robot " + std::to_string(id) + " received message from Robot " + std::to_string(msg.id);
            log_info(log_msg);
        }
    }
}

double Robot::getCurrentTime() {

    return world->getElapsedTime();
}


bool Robot::checkIfNewInfoAvailable() {

    log_info("In checkIfNewInfoAvailable...");

    // Before checking for new info, get rid of any pings that are older than the comms timeout threshold (robot either offline or out of comms)
    clearStalePings(); // Not fully tested yet

    bool info_available = false;

    std::unordered_map<int, std::vector<std::pair<int,double>>>& world_ping_tracker = world->getPingTracker();
    int receiverID = getID(); // can we just do id?

    // just for print >>>
    //printWorldPingTracker(world_ping_tracker);
    // <<< just for print

    // Get pings from last update (i.e., check via this function) saved in last_pings (init'd as empty vector)

    // Find current robot's ping vector
    if (world_ping_tracker.find(receiverID) != world_ping_tracker.end()) {
        std::vector<std::pair<int,double>>& new_pings = world_ping_tracker[receiverID];
        log_info("new_pings: ");
        utils::log1DVector(new_pings, *this);
        log_info("last_pings: ");
        utils::log1DVector(last_pings, *this);

        // Check 1: if a new robot has entered comms - this would qualify as new info
        // Check 2: if a robot was already in comms but has made changes to its belief via cbba since last check - new info here too
        // CHECK 2 IS FLAWED (scope too broad) SO COMMENTED OUT
        for (const auto& ping : new_pings) {
            int other_robot_id = ping.first; // For check 1
            double other_robot_new_timestamp = ping.second; // For check 2 (note timestamp may be the same - denotes last cbba variable update)

            // Check if the robot that sent the received ping had already been heard during the last check
            auto it = std::find_if(last_pings.begin(), last_pings.end(), 
                      [&other_robot_id](const std::pair<int,double>& p) { 
                          return p.first == other_robot_id; });

            // Check 1
            if (it == last_pings.end()) {
                // other robot id not found in last_pings, meaning the robot has newly been heard via ping!
                info_available = true;
                log_info("NEW INFO FOUND DUE TO NEW ROBOT IN COMMS");

                /*// Update last_pings with new_pings
                log_info("New info available");
                last_pings = new_pings;
                log_info("last_pings updated to: ");
                utils::log1DVector(last_pings, *this);

                break; // Stop checking because already found at least one instance of new info*/

            // Check 2 // COMMENTED OUT FOR NOW BECAUSE IT CATCHES INTERNAL CHANGE, NOT JUST EXTERNAL 
            // (causes redundant runs of CBBA since internal belief changes are more common) 
            // Internal meaning CBBA-induced changes between robot i and neighbor robot k
            // External meaning CBBA-induced changes between robot k and neighbors of k, j, where j not a neighbor of i
            } /*else { // Other robot id was found, meaning it was already in comms at last check

                // Let's check if timestamp has changed (i.e., is now higher), indicating other robot has made belief changes via CBBA since last check
                double other_robot_prev_timestamp = it->second; // iterator found the id, timestamp pair, accessing timestamp
                if ( other_robot_prev_timestamp < other_robot_new_timestamp ) {
                    // New timestamp is different (larger) than previous meaning more recent update was made to this other robot's bundle/path/winners list or winning bids list
                    info_available = true;
                    log_info("NEW INFO FOUND DUE TO IN-COMMS ROBOT CBBA SELF-UPDATE");
                }

            }*/

            if (info_available) {

                // Update last_pings with new_pings
                log_info("New info available");
                last_pings = new_pings;
                log_info("last_pings updated to: ");
                utils::log1DVector(last_pings, *this);

                break; // Stop checking because already found at least one instance of new info
            }

        }

    }

    log_info("end checkIfNewInfoAvailable");

    return info_available;
}

void Robot::printWorldPingTracker(std::unordered_map<int, std::vector<std::pair<int,double>>>& world_ping_tracker) {

   for (const auto& pair : world_ping_tracker) {
        int temp_id = pair.first;
        const std::vector<std::pair<int,double>>& pings = pair.second; // pings, which are pairs: sender ID, timestamp of sender's last CBBA bundle/path update

        std::string bla = "Receiver ID: " + std::to_string(temp_id) + " received pings from: ";
        log_info(bla);
        utils::log1DVector(pings,*this);
    }
}

/*void Robot::receivePings() {

    // I don't think we really need this, it is just accessing world ping tracker and print/logging

    //std::cout << "in receivePings........" << std::endl;
    std::unordered_map<int, std::vector<std::pair<int,double>>>& world_ping_tracker = world->getPingTracker();

    int receiverID = id;
    //std::cout << "receiverID: " << receiverID << std::endl;

    printWorldPingTracker(world_ping_tracker);
    
    if (world_ping_tracker.find(receiverID) != world_ping_tracker.end()) {
        //log_info("Found id in world ping tracker");
        std::vector<std::pair<int,double>>& pings = world_ping_tracker[receiverID];
        //utils::log1DVector(pings, *this);
   
        if (!pings.empty()) {
            int first_heard_robot_ID = pings.front().first;
            //std::cout << "Robot " << receiverID << " received a ping from Robot " << first_heard_robot_ID << std::endl;
            std::string log_msg = "Robot " + std::to_string(receiverID) + " received ping from Robot " + std::to_string(first_heard_robot_ID);
            log_info(log_msg);
        }
    }

}*/

void Robot::updateTimestamps() {

    // Not yet tested
    // Timestamps map for current robot i maintains time of last message from each other robot by ID k
    // Sometimes a message won't have been received from another robot k, and this is handled by the indirect timestamp case below
    // The indirect timestamp case checks if a neighbor of robot i has received a message from robot k and if so, updates with that time
    // What about when i = k? Did we prevent or handle this case?

    log_info("Timestamps BEFORE change in robot::updateTimestamps:");
    utils::logUnorderedMap(timestamps,*this);

    bool found_msg_from_k;
    for (auto& [id_k, timestamp] : timestamps) {
        // Check if current robot has new msg directly from k, and if so, update 
        found_msg_from_k = false;
        for (Msg& msg : message_queue) {
            if (msg.id == id_k) {
                // Found message from robot k
                timestamps[id_k] = getCurrentTime(); //msg.timestamp;
                if (timestamps[id_k] < 0.0001) {
                    std::string blop = "Found timestamp of < 0.0001 for id k " + std::to_string(id_k); 
                }
                found_msg_from_k = true;
            }
        }


        // Indirect timestamp case, relayed via some agent m who is in range of i and received a message from k
        if (!found_msg_from_k) { 
            // Robot k not in comms with robot i (current) so check robots in range with both i and k for msg from k
            double new_timestamp = world->getMaxNeighborTimestamp(id,id_k); // latest timestamp of msg from k that a neighbor to i received
            if (timestamps[id_k] < 0.0001) {
                    std::string blop = "Found (in indirect case) timestamp of < 0.0001 for id k " + std::to_string(id_k); 
                }
            if (new_timestamp != -1.0) { // If actually new info
                timestamps[id_k] = new_timestamp;
            }
        }

    }

    log_info("Timestamps AFTER change in robot::updateTimestamps:");
    utils::logUnorderedMap(timestamps,*this);
}

/*void Robot::printMessageQueue(std::vector<Msg>& message_queue) {
    for (const auto& msg : message_queue) {
        printMessage(msg);
    }
}*/

/*void Robot::printMessage(Msg msg) { // no mutex because used within the function above
    std::cout << "Message ID:" << msg.id << "\n";
    //std::cout << "Task ID: " << msg.task_id << "\n";
    //std::cout << "Location: (" << msg.location.x << ", " << msg.location.y << ", " << msg.location.theta << ")\n";
   /* std::cout << "Bundle: [";
    for (const auto& task : msg.bundle.tasks) {
        std::cout << task << " "; // Assuming tasks can be printed this way
    }
    std::cout << "]\n";}*/

bool Robot::needRegroup() {
    //std::cout << " IN REGROUP" << std::endl;
    //std::cout << "message_queue length" << message_queue.size() << std::endl;

    if (!message_queue.empty()) {
        Msg least_recent_msg = message_queue[0];
        //std::cout << "MESSAGE EXISTS from robot ID " << least_recent_msg.id << std::endl;
        //std::cout << "in regroup before true" << std::endl;
        return true;
    }
    //std::cout << "in regroup before false" << std::endl;
    return false;
}

void Robot::countConvergedIterations() {

    // This must be called before updateBeliefs() at the end of a CBBA round

    bool found_difference = foundBeliefUpdate();

   /* // std::vector and std::unordered_map<int,int> support !=
    if (bundle != prev_bundle) {  found_difference = true; }
    if (path != prev_path) { found_difference = true; }
    if (winners != prev_winners) { found_difference = true; }

    // std::unordered_map<int,double> does not
    for (const auto& [task_id, bid] : winning_bids) {
        auto it = prev_winning_bids.find(task_id);
        if (it == prev_winning_bids.end() || std::abs(it->second - bid) > 1e-6) {
            found_difference = true;
        }
    }*/

    if (found_difference) {
        num_converged_iterations = 0;
        //return false;
    } else {     // If get here, no changes found, so system has converged at least temporarily (for the group in comms with current robot now)
        log_info("At convergence for this round!!!!!");
        num_converged_iterations += 1;
        std::string bla = "Converged for " + std::to_string(num_converged_iterations) + " iterations...";
        log_info(bla);
        //return true;
    }

}

bool Robot::foundBeliefUpdate() {

    // Return true if bundle or path or winners or winning bids list have changed in the most recent round of CBBA 

    bool found_difference = false;

    // std::vector and std::unordered_map<int,int> support !=
    if (bundle != prev_bundle) {  found_difference = true; }
    if (path != prev_path) { found_difference = true; }
    if (winners != prev_winners) { found_difference = true; }

    // std::unordered_map<int,double> does not
    for (const auto& [task_id, bid] : winning_bids) {
        auto it = prev_winning_bids.find(task_id);
        if (it == prev_winning_bids.end() || std::abs(it->second - bid) > 1e-6) {
            found_difference = true;
        }
    }

    return found_difference;
}

void Robot::updateBeliefs() {

    // This must be called after convergence is checked for a given round of CBBA

    // Update belief tracking
    prev_bundle = bundle;
    prev_path = path;
    prev_winners = winners;
    prev_winning_bids = winning_bids;

}

void Robot::resetConvergenceCount() {

    num_converged_iterations = 0;

}

void Robot::resetNumCBBARounds() {

    cbba_rounds = 0;

}

void Robot::updateLastSelfUpdateTime(double new_update_timestamp) {

    // Track time that this robot last updated its bundle and/or path

    time_of_last_self_update = new_update_timestamp;
}

void Robot::clearStalePings() {

    double current_time = getCurrentTime();

    std::unordered_map<int, std::vector<std::pair<int,double>>>& world_ping_tracker = world->getPingTracker();

    // Find current robot's ping vector
    if (world_ping_tracker.find(id) != world_ping_tracker.end()) {
        std::vector<std::pair<int,double>>& new_pings = world_ping_tracker[id];

        log_info("current pings vector before clearing any stale pings: ");
        utils::log1DVector(new_pings, *this);

        for (int i = new_pings.size() - 1; i >= 0; i--) { // Backward iteration of pings to prevent issues if/when pings removed

            // Check each ping's timestamp against current time
            double ping_age = current_time - new_pings[i].second;
            if (ping_age > comms_timeout_threshold) {
                int id_of_offline_robot = new_pings[i].first;
                new_pings.erase(new_pings.begin() + i);
                std::string bla = "ERASING A PING THAT IS OLDER THAN THRESHOLD (id of offline robot is " + std::to_string(id_of_offline_robot) + ")";
                log_info(bla);
            }
        }

        log_info("pings vector after removing stale pings: ");
        utils::log1DVector(new_pings, *this);

    }


}

bool Robot::ExploreA() {
    // Condition that should return true when first task in path is Explore_A (will check by ID)

    if (path.empty()) {
        return false;
    }

    // Get info for first task in path (i.e., task that has been allocated to occur next)
    TaskInfo& next_task = world->getTaskInfo(path[0]);

    if (next_task.name == "Explore_A") {
        log_info("Next task to execute is Explore_A!");
        return true;
    }

    return false; // It's not explore A

}

std::pair<int,int> Robot::getNextStartLocation() {

    // Location robot should go to start the current (first) task in the path
    // THIS SHOULD ONLY BE USED FOR SHORTEST PATH DIRECTLY
    // Function below is for getting area if applicable (for coverage tasks)

    return world->getTaskLocation(path[0]); // Either location directly or via area

}