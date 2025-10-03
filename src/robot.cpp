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
Robot::Robot(Planner* p, ShortestPath* sp, CoveragePath* cp, World* w, JSONParser* psr, const Pose2D& initial_pose, int robot_id, std::string robot_type, cv::Scalar dot_color) 
: planner(p), shortest_path(sp), coverage_path(cp), world(w), parser(psr), id(robot_id), type(robot_type) {
    
    // Get filename for logging and then open it
    std::string filename = generateLogFilename();
    std::ofstream clear(filename, std::ios::out); // Clear logging file from previous run
    clear.close();
    robot_log.open(filename, std::ios::app); // Allow appending

    pose = {0, 0, 0};
    //goal = goal_pose; // Like return to home or drop off item loc, specific to each robot
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
    winners = initWinners(); // CBBA
    log_info("Winners (task id: int): ");
    utils::logUnorderedMap(winners, *this);
    winning_bids = initWinningBids(); // CBBA
    log_info("Winning bids (task id: double): ");
    utils::logUnorderedMap(winning_bids, *this);
    winning_bids_matrix = initWinningBidsMatrix(); // CBGA
    log_info("Winning bids matrix: ");
    utils::log2DVector(winning_bids_matrix, *this);
    timestamps = initTimestamps();
    locations = initLocations();

    task_progress = initTaskProgress();

    // Inits for tracking of previous info for convergence checks between iterations
    prev_bundle.resize(max_depth, -1);
    prev_path.resize(max_depth, -1);
    prev_winners = initWinners(); // CBBA
    prev_winning_bids = initWinningBids(); // CBBA
    prev_winning_bids_matrix = initWinningBidsMatrix(); // CBGA
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

    at_consensus = false; // Used to stop executing a path not yet at consensus (relevant for CBBA, not greedy)

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

void Robot::resetBids() {

    // Needed in buildBundle, avoids seg fault when done here?

    bids.clear();
    bids = initBids();
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

std::vector<std::vector<double>> Robot::initWinningBidsMatrix() {

    // Given the number of tasks J and the number of agents N
    // Not every robot can do every task j in J, relevant elements will reflect this by staying 0.0 forever
    // Create winning bids matrix of size row x column = J x N where J is the number of local tasks

    std::string blor = "Num local tasks: " + std::to_string(world->getNumLocalTasks());
    log_info(blor);
    std::string blor2 = "Num agents: " + std::to_string(world->getNumAgents());
    log_info(blor2);

    int J = world->getNumLocalTasks();
    int N = world->getNumAgents();

    std::vector<std::vector<double>> winning_bids_matrix(world->getNumLocalTasks(), std::vector<double>(world->getNumAgents(), 0.0));
    //std::vector<std::vector<double>> winning_bids_matrix(J, std::vector<double>(N, 0.0));

    return winning_bids_matrix;
}



std::unordered_map<int,double> Robot::initTimestamps() {

    std::unordered_map<int,double> timestamps;

    // Get agentinfo from world, just to loop through IDs
    std::unordered_map<int,AgentInfo> all_agents_info = world->getAllAgentsInfo();

    for (const auto& [id_k, agent] : all_agents_info) {
        // For all id's k, not equal to current id i, init timestamp
        if (id_k != id) {
            timestamps[id_k] = -1; 
        }
    }

    return timestamps; 

}

std::unordered_map<int,Pose2D> Robot::initLocations() {

    std::unordered_map<int,Pose2D> locations;

    // Get agentinfo from world
    std::unordered_map<int,AgentInfo> all_agents_info = world->getAllAgentsInfo();

    // Using world agentinfo just to loop through all other robot IDs
    for (const auto& [id_k, agent] : all_agents_info) {
        
        Pose2D unknown = {-1,-1,0};  // Unknown location, orientation theta irrelevant so 0
        locations[id_k] = unknown; // Robot does not know where other robots are until comms

        if (id_k == id) { 
            // Robot knows where it starts
            locations[id_k] = pose;
        }
    }

    log_info("after initLocations");
    utils::logUnorderedMap(locations, *this);

    return locations;
}

std::unordered_map<int,int> Robot::initTaskProgress() {

    std::unordered_map<int,int> task_progress;

    // Get ALL tasks from world, not just doable ones per current robot type
    std::unordered_map<int,TaskInfo>& all_tasks_info = world->getAllTasksInfo();

     for (const auto& task : all_tasks_info) {
        int task_id = task.first;
        task_progress[task_id] = 0; // At start, no tasks started
    }

    return task_progress;

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
    //world->plot(); // Add dots at all robot locations, CAUSES DELAY IF LEFT LIKE THIS (fix below)

    // Robot creates thread just for plotting new position, and detaches to continue with flow of processes other than plotting
    // This fixes the big delay that was happening for one of the robots, not always the same one
    // Needed consistent concurrent movement to fairly represent real world (albeit distantly)
    // and plot wrt time in such a way that delays are due to task allocation choices, not behind the scenes delays
    std::thread([this]() { world->plot(); }).detach();

    double drain_percent = 0.01;
    updateBatteryLevel(drain_percent);
    //std::cout << "New battery level after move: " << battery_level << " for robot ID " << id << std::endl;
}

// debug version of move with prints below
/*void Robot::move(Pose2D waypoint) {
    auto move_start = std::chrono::high_resolution_clock::now();
    
    auto clear_start = std::chrono::high_resolution_clock::now();
    world->clear(pose);
    auto clear_end = std::chrono::high_resolution_clock::now();
    
    auto pose_start = std::chrono::high_resolution_clock::now();
    pose = waypoint;
    auto pose_end = std::chrono::high_resolution_clock::now();
    
    auto plot_start = std::chrono::high_resolution_clock::now();
    // world->plot(); // testing with world on its own thread, created and run in main
    std::thread([this]() { world->plot(); }).detach(); // pt 2 of crucial fix (other part in world.plot mutex scope lessened)
    auto plot_end = std::chrono::high_resolution_clock::now();
    
    auto battery_start = std::chrono::high_resolution_clock::now();
    double drain_percent = 0.01;
    updateBatteryLevel(drain_percent);
    auto battery_end = std::chrono::high_resolution_clock::now();
    
    auto move_end = std::chrono::high_resolution_clock::now();
    
    // Calculate timings
    double clear_time = std::chrono::duration<double>(clear_end - clear_start).count();
    double pose_time = std::chrono::duration<double>(pose_end - pose_start).count();
    double plot_time = std::chrono::duration<double>(plot_end - plot_start).count();
    double battery_time = std::chrono::duration<double>(battery_end - battery_start).count();
    double total_time = std::chrono::duration<double>(move_end - move_start).count();
    
    std::string quorp = "Robot " + std::to_string(id) + " move breakdown - Clear: " + 
             std::to_string(clear_time) + "s, Pose: " + std::to_string(pose_time) + 
             "s, Plot: " + std::to_string(plot_time) + "s, Battery: " + 
             std::to_string(battery_time) + "s, Total: " + std::to_string(total_time) + "s";
    log_info(quorp);
}*/

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
    //std::lock_guard<std::mutex> lock(world->getWorldMutex()); tested with unsafe (no mutex) func below, didn't help
    //std::cout << "in receiveMessages........" << std::endl;
    std::unordered_map<int, std::vector<Msg>>& world_msg_tracker = world->getMessageTracker(); // can't be used with mutex in this function
    //std::unordered_map<int,std::vector<Msg>>& world_msg_tracker = world->getMessageTrackerUnsafe();// testing this with mutex added above, didn't help

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

        log_info("before receiving messages officially, msg tracker for current robot is: ");
        // utils::log1DVector(messages, *this);
        utils::logMsgVector(messages, *this);

        // Sort messages to keep same order, more determinism (idk leaving out again because its not technically wrong to have nondeterministic order)
        // std::sort(messages.begin(), messages.end(), 
        //           [](const Msg& a, const Msg& b) { return a.id < b.id; });

        // Process all messages for receiver robot queued in world message tracker
        for (const auto& msg : messages) {
            updateRobotMessageQueue(msg);
            int id_k = msg.id;
            updateTimestampGivenDirectMessage(id_k);
            std::string log_msg = "Robot " + std::to_string(id) + " received message from Robot " + std::to_string(msg.id);
            log_info(log_msg);
        }

        updateRemainingTimestampsIndirectly();

        // New clearing of messages in world tracker for this robot since fully received by robot
        messages.clear();

        log_info("after receiving messages officially and clearing, msg tracker for current robot is: ");
        // utils::log1DVector(messages, *this);
        utils::logMsgVector(messages, *this);

        // Old way: only processed one message from queue
        // if (!messages.empty()) {
        //     Msg msg = messages.front();


        //     /////////////messages.erase(messages.begin());  // Remove the message after reading // COMMENTED OUT FOR TEST



        //     //timestamps[msg.id] = getCurrentTime(); this is done in updateTimestamps
        //     // log_info("in receiveMessages!!!!");
        //     // log_info("Size of world messages queue for current robot: ");
        //     // log_info(std::to_string(messages.size()));
        //     // log_info("Winners: ");
        //     // utils::logUnorderedMap(msg.winners,*this);
        //     // log_info("Winning bids: ");
        //     // utils::logUnorderedMap(msg.winning_bids,*this);
        //     // log_info("Timestamps: ");
        //     // utils::logUnorderedMap(msg.timestamps,*this);
        //     updateRobotMessageQueue(msg);
        //     //std::cout << "Robot " << getID() << " received a message from Robot " << msg.id << std::endl;
        //     std::string log_msg = "Robot " + std::to_string(id) + " received message from Robot " + std::to_string(msg.id);
        //     log_info(log_msg);
        // }
    }

    std::cout << "end receiveMessages" << std::endl;
}

double Robot::getCurrentTime() {

    return world->getElapsedTime();
}


bool Robot::checkIfNewInfoAvailable() {

    log_info("In checkIfNewInfoAvailable...");

    // Before checking for new info, get rid of any pings that are older than the comms timeout threshold (robot either offline or out of comms)
    clearStalePings();

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

// first half of old updateTimestamps(), splitting for testing calls in different places 
void Robot::updateTimestampGivenDirectMessage(int id_k) {
    timestamps[id_k] = getCurrentTime(); //msg.timestamp;
    if (timestamps[id_k] < 0.0001) {
        std::string blop = "Found timestamp of < 0.0001 for id k " + std::to_string(id_k); 
    }
}

// second half of old updateTimestamps(), splitting for testing calls in different places 
void Robot::updateRemainingTimestampsIndirectly() {

    bool found_msg_from_k;
    for (auto& [id_k, timestamp] : timestamps) {

        // Determine whether robot i has direct message from k in queue
        found_msg_from_k = false;
        for (Msg& msg : message_queue) {
            if (msg.id == id_k) {
                // Found message from robot k
                found_msg_from_k = true;
            }
        }

        // Indirect timestamp case, robot i has no message direct from k, so info relayed via some agent m who is in range of i and received a message from k
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

}

void Robot::updateTimestamps() {

    // Timestamps map for current robot i maintains time of last message from each other robot by ID k
    // Sometimes a message won't have been received from another robot k, and this is handled by the indirect timestamp case below
    // The indirect timestamp case checks if a neighbor of robot i has received a message from robot k and if so, updates with that time
    // What about when i = k? Did we prevent or handle this case?

    // log_info("Timestamps BEFORE change in robot::updateTimestamps:");
    // utils::logUnorderedMap(timestamps,*this);

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

    log_info("Updated Timestamps:");
    utils::logUnorderedMap(timestamps,*this);
}

void Robot::updateLocations() {

    log_info("locations update start");
    std::string blopr = "number of msgs in queue: " + std::to_string(message_queue.size());
    log_info(blopr);

    for (auto& [id_k, location] : locations) { // int, Pose2D

        std::string b = "k: " + std::to_string(id_k);
        log_info(b);

        if (id_k == id) { // Defer to self for own location
            locations[id] = pose; // Check that pose is actually updating correctly
            std::string blorg = "Defer to own location: " + std::to_string(pose.x) + "," 
                + std::to_string(pose.y) + "," + std::to_string(pose.theta);
            log_info(blorg);
        } else { 
            std::string bla = "k is not i, k is: " + std::to_string(id_k);
            log_info(bla);
            // Find id and locations vector from msg of robot that has most recent timestamp for robot k
            std::pair<int, Pose2D> best_info = getMostUpToDateNeighborInfo(id_k);
            int id_m = best_info.first;
            Pose2D most_recent_k_location_from_m = best_info.second; // According to m
            std::string blarg = "best id m: " + std::to_string(id_m);
            log_info(blarg);
            std::string blurg = "Defer to m belief of k location: " + std::to_string(most_recent_k_location_from_m.x) + "," 
                + std::to_string(most_recent_k_location_from_m.y) + "," + std::to_string(most_recent_k_location_from_m.theta);
            log_info(blurg);

            // Defer to m (if m = id of self, defering to self means no changes)
            if (id_m != id) {
                log_info("in update for m (m != i) with best info on k loc");
                locations[id_k] = most_recent_k_location_from_m;
            }

        }
    }

    log_info("After locations update");
    utils::logUnorderedMap(locations, *this);

}

std::pair<int, Pose2D> Robot::getMostUpToDateNeighborInfo(int id_k) {

    // Find which neighbor has communicated with k most recently, and thus has most recent k location

    std::pair<int, Pose2D> best_info;

    // Init winner, aka robot with most up-to-date info on robot k, assume it is self
    int defer_to_id = id;
    double max_k_timestamp = timestamps[id_k];
    Pose2D most_recent_k_location = locations[id_k];
    std::string blorg = "k loc according to i: " + std::to_string(most_recent_k_location.x) + "," 
                + std::to_string(most_recent_k_location.y) + "," + std::to_string(most_recent_k_location.theta);
    log_info(blorg);

    for (Msg& msg : message_queue) {
        int id_m = msg.id; // Will never be self id because self doesn't send msg to self

        if (id_m == id_k) {
            best_info.first = id_m;
            best_info.second = msg.locations[id_k]; // k location according to itself
            log_info("msg.locations where m=k: ");
            utils::logUnorderedMap(msg.locations, *this);
            std::string blorg2 = "k loc according to k: " + std::to_string(best_info.second.x) + "," 
                + std::to_string(best_info.second.y) + "," + std::to_string(best_info.second.theta);
            log_info(blorg2);
            return best_info; // Always defer to k about its own location

        } else if (msg.timestamps[id_k] > max_k_timestamp) { // Found robot m (where m != k) with more recent info on k than current winner
            // Update winner
            defer_to_id = id_m;
            max_k_timestamp = msg.timestamps[id_k];
            most_recent_k_location = msg.locations[id_k];
            std::string blorg3 = "k loc according to best m: " + std::to_string(most_recent_k_location.x) + "," 
                + std::to_string(most_recent_k_location.y) + "," + std::to_string(most_recent_k_location.theta);
            log_info(blorg3);

        }
    }

    best_info.first = defer_to_id;
    best_info.second = most_recent_k_location;

    return best_info;
}


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

void Robot::countConvergedIterations(bool do_cbga) {

    // This must be called before updateBeliefs() at the end of a CBBA round

    bool found_difference = foundBeliefUpdate(do_cbga);

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

bool Robot::foundBeliefUpdate(bool do_cbga) {

    // Return true if bundle or path or winners or winning bids list have changed in the most recent round of CBBA 

    bool found_difference = false;

    // std::vector and std::unordered_map<int,int> support !=
    if (bundle != prev_bundle) {  found_difference = true; }
    if (path != prev_path) { found_difference = true; }

    if (do_cbga) {

        for (size_t i = 0; i < winning_bids_matrix.size() && !found_difference; i++) {
            for (size_t j = 0; j < winning_bids_matrix[i].size(); j++) {
                if (std::abs(winning_bids_matrix[i][j] - prev_winning_bids_matrix[i][j]) > 1e-6) {
                    found_difference = true;
                    break;
                }
            }
        }

    } else { // cbba, winners and winning_bids
        if (winners != prev_winners) { found_difference = true; }

        for (const auto& [task_id, bid] : winning_bids) {
            auto it = prev_winning_bids.find(task_id);
            if (it == prev_winning_bids.end() || std::abs(it->second - bid) > 1e-6) {
                found_difference = true;
            }
        }

    }
    

    return found_difference;
}

void Robot::updateBeliefs(bool do_cbga) {

    // This must be called after convergence is checked for a given round of CBBA

    // Update belief tracking
    prev_bundle = bundle;
    prev_path = path;

    if (do_cbga) {
        prev_winning_bids_matrix = winning_bids_matrix;
    } else {
        prev_winners = winners;
        prev_winning_bids = winning_bids;
    }

}

void Robot::resetConvergenceCount() {

    num_converged_iterations = 0;

}

void Robot::resetNumCBBARounds() {

    cbba_rounds = 0;

}

void Robot::resetNumCBGARounds() {

    cbga_rounds = 0;

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

    // Can combine these false ifs later if kept

    log_info("in robot ExploreA function");

    if (!at_consensus) { // prevent triggering the start of a new action's execution if task allocation is in progress
        return false;
    }

    log_info("in robot ExploreA function 2");

    if (path.empty()) {
        return false;
    }

    log_info("in robot ExploreA function 3");

    if (!world->hasTaskInfo(path[0])) {
        log_info("world not done initializing...");
        return false;  // World not done initializing task info 
    }

    // Get info for first task in path (i.e., task that has been allocated to occur next)
    TaskInfo& next_task = world->getTaskInfo(path[0]);

    log_info("next task (in ExploreA): ");
    log_info(next_task.name);

    if (next_task.name == "Explore_A") {
        log_info("Next task to execute is Explore_A!");
        return true;
    }

    return false; // It's not explore A

}

bool Robot::ExploreB() {
    // Condition that should return true when first task in path is Explore_A (will check by ID)

    if (!at_consensus) { // prevent triggering the start of a new action's execution if task allocation is in progress
        return false;
    }

    if (path.empty()) {
        return false;
    }

    if (!world->hasTaskInfo(path[0])) {
        return false;  // World not done initializing task info 
    }

    // Get info for first task in path (i.e., task that has been allocated to occur next)
    TaskInfo& next_task = world->getTaskInfo(path[0]);

    if (next_task.name == "Explore_B") {
        log_info("Next task to execute is Explore_B!");
        return true;
    }

    return false; // It's not explore B

}

bool Robot::ExploreC() {
    // Condition that should return true when first task in path is Explore_A (will check by ID)

    if (!at_consensus) { // prevent triggering the start of a new action's execution if task allocation is in progress
        return false;
    }

    if (path.empty()) {
        return false;
    }

    if (!world->hasTaskInfo(path[0])) {
        return false;  // World not done initializing task info 
    }

    // Get info for first task in path (i.e., task that has been allocated to occur next)
    TaskInfo& next_task = world->getTaskInfo(path[0]);

    if (next_task.name == "Explore_C") {
        log_info("Next task to execute is Explore_C!");
        return true;
    }

    return false; // It's not explore B

}

bool Robot::ExploreD() {
    // Condition that should return true when first task in path is Explore_A (will check by ID)

    if (!at_consensus) { // prevent triggering the start of a new action's execution if task allocation is in progress
        return false;
    }

    if (path.empty()) {
        return false;
    }

    if (!world->hasTaskInfo(path[0])) {
        return false;  // World not done initializing task info 
    }

    // Get info for first task in path (i.e., task that has been allocated to occur next)
    TaskInfo& next_task = world->getTaskInfo(path[0]);

    if (next_task.name == "Explore_D") {
        log_info("Next task to execute is Explore_D!");
        return true;
    }

    return false; // It's not explore B

}

std::pair<int,int> Robot::getNextStartLocation() {

    // Location robot should go to start the current (first) task in the path
    // THIS SHOULD ONLY BE USED FOR SHORTEST PATH DIRECTLY
    // Function below is for getting area if applicable (for coverage tasks)

    return world->getTaskLocation(path[0]); // Either location directly or via area

}

void Robot::removeCompletedTaskFromPath() {
    if (path.empty()) {
        log_info("Warning: Attempted to remove from empty path");
        return;
    }

    // Remove from path
    path.erase(path.begin());
    path.push_back(-1); // Maintain same size as bundle but with -1's for empty
    log_info("Path is now: ");
    utils::log1DVector(path, *this);
}



std::unordered_map<std::string, int> Robot::initTaskGroupFullnessMap() {

    // Assume empty, init with zeros

    std::unordered_map<std::string, int> task_group_fullness;

    std::vector<std::string> agent_types = world->getAgentTypes();

    for (auto& type : agent_types) {
        task_group_fullness[type] = 0;
    }

    return task_group_fullness;
}

// testing
// std::vector<int> Robot::getAssignedAgents(int task_id) {
//     log_info("start getAssignedAgents()");

//     std::vector<int> assigned_agent_ids;
    
//     // Get matrix reference once
//     const std::vector<std::vector<double>>& winning_bids_matrix = getWinningBidsMatrix();
    
//     // Validate dimensions
//     int current_tasks = world->getNumLocalTasks();
//     int current_agents = world->getNumAgents();
//     int task_idx = task_id - 1;
    
//     std::cout << "Matrix check: " << winning_bids_matrix.size() << "x" 
//               << (winning_bids_matrix.empty() ? 0 : winning_bids_matrix[0].size())
//               << " vs World: " << current_tasks << "x" << current_agents << std::endl;
    
//     // Bounds checking
//     if (task_idx < 0 || task_idx >= winning_bids_matrix.size()) {
//         std::cout << "ERROR: task_idx=" << task_idx << " out of bounds!" << std::endl;
//         return assigned_agent_ids;
//     }
    
//     if (current_agents > winning_bids_matrix[task_idx].size()) {
//         std::cout << "ERROR: num_agents=" << current_agents 
//                   << " > matrix columns=" << winning_bids_matrix[task_idx].size() << std::endl;
//         return assigned_agent_ids;
//     }
    
//     // Check for assignments
//     std::cout << "Checking task " << task_id << " (idx=" << task_idx << ")" << std::endl;
//     for (int a = 0; a < current_agents; a++) {
//         if (winning_bids_matrix[task_idx][a] != 0) {
//             int agent_id = a + 1;
//             assigned_agent_ids.push_back(agent_id);
//             std::cout << "Found assigned agent: " << agent_id << std::endl;
//         }
//     }
    
//     std::cout << "Found " << assigned_agent_ids.size() << " assigned agents" << std::endl;
//     log_info("end getAssignedAgents()");
//     return assigned_agent_ids;
// }

// original
std::vector<int> Robot::getAssignedAgents(int task_id) {

    // Gets any robots assigned to task current robot is considering assigning to itself (SO NOT INCLUDING CURRENT)
    // Basic testing done (in cbga, will double check still working when called from here with minor changes (equivalent logic))
    // When new testing done, say so here

    log_info("start getAssignedAgents()");

    std::vector<int> assigned_agent_ids;

    // CONVERT TO INDEX: To accomodate winning bid matrix row 0 being for task 1, etc. (same reason we use a and not a+1 for indexing below)
    int task_idx = task_id-1; 

    std::vector<std::vector<double>>& winning_bids_matrix = getWinningBidsMatrix();
    log_info("winning_bids_matrix: ");
    utils::log2DVector(winning_bids_matrix, *this);

    // BELOW COMMENTED OUT BIT IS FOR TEST ONLY (this was tested with function in cbga class)
    /*log_info("ADDING agent assignment to co-op task, id 5 (idx 4), for TEST (see robot 1 logs only)");
    winning_bids_matrix[4][1] = 2.0; // assigning agent 2 (at idx 1)
    utils::log2DVector(winning_bids_matrix, this);*/

    // For each agent with nonzero winning bid for given task
    int num_agents = world->getNumAgents();
    for (int a = 0; a < num_agents; a++) {

        std::string ap = "a: " + std::to_string(a);
        log_info(ap);

        //robot.log_info(std::to_string(a));
        std::string blop = "winning bid element for (" + std::to_string(task_idx) + "," + std::to_string(a) + "): " + std::to_string(winning_bids_matrix[task_idx][a]);
        log_info(blop);
        
        if (winning_bids_matrix[task_idx][a] != 0) { // if task_id, agent_id has nonzero winning bid, then that agent is assigned to that task

            int agent_id = a+1; // CONVERT TO AGENT ID, assumed to be +1 since never agent 0, starts at agent 1
            assigned_agent_ids.push_back(agent_id);
        }

    }

    log_info("end getAssignedAgents()");
    return assigned_agent_ids;
}

std::unordered_map<std::string, std::vector<int>> Robot::initTaskSubGroupMap() {

    // Tested

    // Assume empty, init with vectors

    std::unordered_map<std::string, std::vector<int>> task_sub_groups;

    std::vector<std::string> agent_types = world->getAgentTypes();

    for (auto& type : agent_types) {
        task_sub_groups[type] = {};
    }

    return task_sub_groups;
}

std::unordered_map<std::string, std::vector<int>> Robot::trackAssignedRobotsbySubGroup(int task_id) {

    // Checks types of all assigned robots and divides them by sub-group
    // Allows for the explicit tracking of robots that should legit be in the "any_agent_type" sub-group
    // Tested

    std::unordered_map<std::string, std::vector<int>> task_sub_groups = initTaskSubGroupMap();

    std::unordered_map<std::string, int>& task_group_max_size = world->getTaskGroupInfo(task_id);

    int task_idx = task_id-1;

    // for each agent assigned to task, get type and count 
    std::vector<int> assigned_agent_ids = getAssignedAgents(task_id);
    log_info("Assigned agent ids (in trackAssignedRobotsbySubGroup):");
    utils::log1DVector(assigned_agent_ids, *this);
    for ( auto& assigned_agent_id : assigned_agent_ids ) {
        std::string agent_type = world->getAgentType(assigned_agent_id);

        // First count agents toward their own type group
        if (task_sub_groups[agent_type].size() < task_group_max_size[agent_type]) {
            task_sub_groups[agent_type].push_back(assigned_agent_id); // Add agent to its type sub-group since there is still room

        } else if (task_group_max_size["any_agent_type"] > 0 && task_sub_groups["any_agent_type"].size() < task_group_max_size["any_agent_type"]) {
            // if already reached limit of required agents of current type, and "any_agent_type" group is assignable to (max > 0 and not full)
            task_sub_groups["any_agent_type"].push_back(assigned_agent_id);
        }
    }

    return task_sub_groups;
}

bool Robot::PathClearingNeeded() {
    // Condition that should return true when first task in path is Clear_Path (will check by ID)

    if (!at_consensus) { // prevent triggering the start of a new action's execution if task allocation is in progress
        return false;
    }

    if (path.empty()) {
        return false;
    }

    if (!world->hasTaskInfo(path[0])) {
        return false;  // World not done initializing task info 
    }

    // Get info for first task in path (i.e., task that has been allocated to occur next)
    TaskInfo& next_task = world->getTaskInfo(path[0]);

    if (next_task.name == "Clear_Path") {
        log_info("Next task to execute is Clear_Path!");
        return true;
    }

    return false; // It's not ClearPath

}

void Robot::updateSingleTaskProgress(int task_id, int started) {

    log_info("in updateSingleTaskProgress");

    // Leaving value as int (not bool) so we can make it 2 as well as 0 and 1 in future to denote complete if needed
    // For now started = 0 means not started, and 1 is started (could be complete)

    task_progress[task_id] = started;


}

void Robot::updateTaskProgress() {

    log_info("in updateTaskProgress");

    for (Msg& msg : message_queue) {

        // started = 1 if started and 0 otherwise (might be 2 for complete later)
        for (const auto& [task_id, started] : msg.task_progress) {
            if (started == 1) {
                // Neighbor knows this task has been started (either by itself or another neighbor)
                task_progress[task_id] = 1;
            }
        }

        // This way we keep local started info (1s), and included knowledge of started tasks that is known by neighbors (updates to 1 above)
    }

    log_info("Task progress in updateTaskProgress: ");
    utils::logUnorderedMap(task_progress, *this);
}

bool Robot::taskAlreadyStarted(int task_id) {

    if (task_progress[task_id] == 1) {

        return true;
    }

    return false;
}