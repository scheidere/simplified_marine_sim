#include <cstdio>
#include <stdexcept> // For exception handling
#include "planners.hpp"
#include "scorer.hpp"
#include "world.hpp"
#include "robot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/actions/pop_from_queue.hpp"
#include "parser.hpp"


// Create state class, which will contain robot condition functions

// Note we init winning_bids and winning_agent_indices with numTasks of 1 because of access issues to numTasks during initialization
Robot::Robot(Planner* p, ShortestPath* sp, CoveragePath* cp, Scorer* s, World* w, JSONParser* psr, const Pose2D& initial_pose, const Pose2D& goal_pose, std::vector<Task> tasks, int robot_id, cv::Scalar dot_color) 
: planner(p), shortest_path(sp), coverage_path(cp), scorer(s), world(w), parser(psr), goal(goal_pose), id(robot_id), winning_bids(1), winning_agent_indices(1) {
    pose = {0, 0, 0};
    goal = goal_pose; // Like return to home or drop off item loc, specific to each robot
    color = dot_color;
    id = robot_id; // Robot ID
    task_id = 0; // ID of current task, have zero represent undefined
    world->trackRobot(this);
    battery_level = 1.0;
    init(initial_pose);
    assignable_tasks = tasks; // Tasks that can be assigned to this robot

    std::cout << "Robot constructor: Getting all tasks from world..." << std::endl;
    std::vector<Task> allTasks = world->getAllTasks();
    int numTasks = allTasks.size();
    std::cout << "Number of tasks in world: " << numTasks << std::endl;

    // Sanity check on task size
    if (numTasks > 1000000) {
        std::cerr << "Warning: Number of tasks is unusually large: " << numTasks << std::endl;
    }

    // Get filename for logging and then open it
    std::string filename = generateLogFilename();
    std::ofstream clear(filename, std::ios::out); // Clear logging file from previous run
    clear.close();
    robot_log.open(filename, std::ios::app); // Allow appending
    //std::string log_msg = "testing log_msg";
    //log(log_msg);

    try {
        std::cout << "Allocating winning_bids and winning_agent_indices vectors..." << std::endl;
        winning_bids = WinningBids(numTasks);
        winning_agent_indices = WinningAgentIndices(numTasks);
    } catch (const std::exception& e) {
        std::cerr << "Exception caught during vector allocation in constructor: " << e.what() << std::endl;
        throw;
    }
    
    initializeWinningBidsAndIndices();
}

std::string Robot::generateLogFilename() {
    std::ostringstream oss;
    oss << "robot_" << id << "_log.txt";  // Create a string in the format "robot_<robotID>_log.txt"
    return oss.str();
}

void Robot::log(std::string log_msg) {

    if (robot_log.is_open()) {
        //std::cerr << "LOG FILE IS OPEN FOR ROBOT " << id << std::endl;

        //robot_log << "Testing: " << id << std::endl;
        robot_log << log_msg << std::endl;
        //robot_log.flush(); // Write immediately
        //robot_log.close();

    } else {
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Failed to open log file for Robot ID " << id << std::endl;
    }

}

void Robot::initializeWinningBidsAndIndices() {
    std::cout << "Initializing winning bids and indices..." << std::endl;
    std::vector<Task> allTasks = world->getAllTasks();
    int numTasks = allTasks.size();

    // Sanity check on task size
    if (numTasks > 1000000) {
        std::cerr << "Warning: Number of tasks is unusually large: " << numTasks << std::endl;
    }

    try {
        std::cout << "Allocating vectors in initializeWinningBidsAndIndices..." << std::endl;
        winning_bids = WinningBids(numTasks);
        winning_agent_indices = WinningAgentIndices(numTasks);
    } catch (const std::exception& e) {
        std::cerr << "Exception caught during vector allocation in initializeWinningBidsAndIndices: " << e.what() << std::endl;
        throw;
    }
}

void Robot::init (Pose2D initial_pose) {
    std::cout << "Initializing robot pose..." << std::endl;
    // Access world for image
    cv::Mat image = world->getImage();
    cv::circle(image, cv::Point(initial_pose.x, initial_pose.y), 5, color, -1);
    pose = initial_pose; // Set current robot pose variable
}

void Robot::printTasksVector() {
    std::cout << "TESTING TASK VECTOR PRINT" << std::endl;
    std::cout << "Number of assignable tasks: " << assignable_tasks.size() << std::endl;

    for (const auto& task : assignable_tasks) {
        task.print();
    }
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
            updateRobotMessageQueue(msg);
            std::cout << "Robot " << getID() << " received a message from Robot " << msg.id << std::endl;
            std::string log_msg = "Robot " + std::to_string(id) + " received message from Robot " + std::to_string(msg.id);
            log(log_msg);
        }
    }
}

void Robot::printMessageQueue(std::vector<Msg>& message_queue) {
    for (const auto& msg : message_queue) {
        printMessage(msg);
    }
}

void Robot::printMessage(Msg msg) { // no mutex because used within the function above
    std::cout << "Message ID:" << msg.id << "\n";
    std::cout << "Task ID: " << msg.task_id << "\n";
    std::cout << "Location: (" << msg.location.x << ", " << msg.location.y << ", " << msg.location.theta << ")\n";
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