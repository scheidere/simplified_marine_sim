#include <cstdio>
#include <stdexcept> // For exception handling
#include "planners.hpp"
#include "scorer.hpp"
#include "world.hpp"
#include "robot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/actions/pop_from_queue.hpp"
#include "parser.hpp"
#include "utils.hpp"


// Create state class, which will contain robot condition functions

// Note we init winning_bids and winning_agent_indices with numTasks of 1 because of access issues to numTasks during initialization
Robot::Robot(Planner* p, ShortestPath* sp, CoveragePath* cp, Scorer* s, World* w, JSONParser* psr, const Pose2D& initial_pose, const Pose2D& goal_pose, int robot_id, std::string robot_type, cv::Scalar dot_color) 
: planner(p), shortest_path(sp), coverage_path(cp), scorer(s), world(w), parser(psr), id(robot_id), type(robot_type) {
    
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
    scores.resize(max_depth, 0.0);

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
    std::cout << "In move for robot with ID " << getID() << std::endl;

    world->clear(pose); // Clear all robot dots (technically only need to clear moving ones, but this is easier)
    std::cout << "Before: ";
    world->printTrackedRobots();
    pose = waypoint; // Update x and y within robot class
    std::cout << "After: ";
    world->printTrackedRobots();
    world->plot(); // Add dots at all robot locations

    double drain_percent = 0.01;
    updateBatteryLevel(drain_percent);
    std::cout << "New battery level after move: " << battery_level << " for robot ID " << id << std::endl;
}

void Robot::updateRobotMessageQueue(Msg msg) {
    std::cout << "IN updateRobotMessageQueue len message_queue: " << message_queue.size() << std::endl;
    try {
        message_queue.push_back(msg);
    } catch (const std::exception& e) {
        std::cerr << "Exception caught while updating message queue: " << e.what() << std::endl;
        throw;
    }
    std::cout << "END updateRobotMessageQueue len message_queue: " << message_queue.size() << std::endl;
}

void Robot::receiveMessages() {
    std::cout << "in receiveMessages........" << std::endl;
    std::unordered_map<int, std::vector<Msg>>& world_msg_tracker = world->getMessageTracker();

    int receiverID = getID();
    std::cout << "receiverID: " << receiverID << std::endl;
    //std::cout << "Printed msg tracker from world: " << std::endl;
    //world->printMessageTracker();
    //std::cout << "world msg tracker printed above, size = " << world_msg_tracker.size() << std::endl;

    if (world_msg_tracker.find(receiverID) != world_msg_tracker.end()) {
        std::cout << "in if" << std::endl;
        std::vector<Msg>& messages = world_msg_tracker[receiverID];
        std::cout << "length of messages from world (should be 1?): " << messages.size() << std::endl;
        std::cout << "!messages.empty(): " << !messages.empty() << std::endl;

        if (!messages.empty()) {
            Msg msg = messages.front();
            //timestamps[msg.id] = getMessageReceptionTime(); this is done in updateTimestamps
            updateRobotMessageQueue(msg);
            std::cout << "Robot " << getID() << " received a message from Robot " << msg.id << std::endl;
            std::string log_msg = "Robot " + std::to_string(id) + " received message from Robot " + std::to_string(msg.id);
            log_info(log_msg);
        }
    }
}

double Robot::getMessageReceptionTime() {

    return world->getElapsedTime();
}

bool Robot::checkIfNewInfoAvailable() {

    log_info("In checkIfNewInfoAvailable...");

    bool info_available = false;

    std::unordered_map<int, std::vector<int>>& world_ping_tracker = world->getPingTracker();
    int receiverID = getID();

    // just for print >>>
    printWorldPingTracker(world_ping_tracker);
    // <<< just for print

    if (world_ping_tracker.find(receiverID) != world_ping_tracker.end()) { // Find current robot's ping vector
        //log_info("Found current robot in ping tracker");
        std::vector<int>& pings = world_ping_tracker[receiverID];
        if (!pings.empty()) {
            info_available = true;
            log_info("New info available");
        }
    }

    log_info("end checkIfNewInfoAvailable");

    return info_available;
}

void Robot::printWorldPingTracker(std::unordered_map<int, std::vector<int>>& world_ping_tracker) {

   for (const auto& pair : world_ping_tracker) {
        int temp_id = pair.first;
        const std::vector<int>& pings = pair.second;

        std::string bla = "Receiver ID: " + std::to_string(temp_id) + " received pings from: ";
        log_info(bla);
        utils::log1DVector(pings,*this);
    }
}

void Robot::receivePings() {

    std::cout << "in receivePings........" << std::endl;
    std::unordered_map<int, std::vector<int>>& world_ping_tracker = world->getPingTracker();

    int receiverID = id;
    std::cout << "receiverID: " << receiverID << std::endl;

    printWorldPingTracker(world_ping_tracker);
    
    if (world_ping_tracker.find(receiverID) != world_ping_tracker.end()) {
        //log_info("Found id in world ping tracker");
        std::vector<int>& pings = world_ping_tracker[receiverID];
        //utils::log1DVector(pings, *this);
   
        if (!pings.empty()) {
            int first_heard_robot_ID = pings.front();
            std::cout << "Robot " << receiverID << " received a ping from Robot " << first_heard_robot_ID << std::endl;
            std::string log_msg = "Robot " + std::to_string(receiverID) + " received ping from Robot " + std::to_string(first_heard_robot_ID);
            log_info(log_msg);
        }
    }

}

void Robot::updateTimestamps() {

    // Not yet tested

    log_info("Timestamps BEFORE change:");
    utils::logUnorderedMap(timestamps,*this);

    bool found_msg_from_k;
    for (auto& [id_k, timestamp] : timestamps) {
        // Check if current robot has new msg directly from k, and if so, update 
        found_msg_from_k = false;
        for (Msg& msg : message_queue) {
            if (msg.id == id_k) {
                // Found message from robot k
                timestamps[id_k] = getMessageReceptionTime(); //msg.timestamp;
                found_msg_from_k = true;
            }
        }


        // Indirect timestamp case, relayed via some agent m who is in range of i and received a message from k
        if (!found_msg_from_k) { 
            // Robot k not in comms with robot i (current) so check robots in range with both i and k for msg from k
            double new_timestamp = world->getMaxNeighborTimestamp(id,id_k); // latest timestamp of msg from k that a neighbor to i received
            if (new_timestamp != -1.0) { // If actually new info
                timestamps[id_k] = new_timestamp;
            }
        }

    }

    log_info("Timestamps AFTER change:");
    utils::logUnorderedMap(timestamps,*this);
}

void Robot::printMessageQueue(std::vector<Msg>& message_queue) {
    for (const auto& msg : message_queue) {
        printMessage(msg);
    }
}

void Robot::printMessage(Msg msg) { // no mutex because used within the function above
    std::cout << "Message ID:" << msg.id << "\n";
    //std::cout << "Task ID: " << msg.task_id << "\n";
    //std::cout << "Location: (" << msg.location.x << ", " << msg.location.y << ", " << msg.location.theta << ")\n";
   /* std::cout << "Bundle: [";
    for (const auto& task : msg.bundle.tasks) {
        std::cout << task << " "; // Assuming tasks can be printed this way
    }
    std::cout << "]\n";*/
}

bool Robot::needRegroup() {
    std::cout << " IN REGROUP" << std::endl;
    std::cout << "message_queue length" << message_queue.size() << std::endl;

    if (!message_queue.empty()) {
        Msg least_recent_msg = message_queue[0];
        std::cout << "MESSAGE EXISTS from robot ID " << least_recent_msg.id << std::endl;
        std::cout << "in regroup before true" << std::endl;
        return true;
    }
    std::cout << "in regroup before false" << std::endl;
    return false;
}

