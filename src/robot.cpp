#include <cstdio>
#include "planners.hpp"
#include "scorer.hpp"
#include "robot.hpp"
#include "world.hpp"
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/actions/pop_from_queue.hpp"


// Create state class, which will contain robot condition functions

Robot::Robot(Planner* p, ShortestPath* sp, Scorer* s, World* w, const Pose2D& goal_pose, int robot_id, cv::Scalar dot_color) 
: planner(p), shortest_path(sp), scorer(s), world(w), goal(goal_pose), id(robot_id) {
    pose = {0, 0, 0};
    goal = goal_pose; // Like return to home or drop off item loc, specific to each robot
    color = dot_color;
    id = robot_id; // Robot ID
    task_id = 0; // ID of current task, have zero represent undefined
    world->trackRobot(this);
    battery_level = 1.0;
    tasks.init()


    // Add current task ID
    // Where will we record things like at_place, in_comms, battery_low, etc.?
    // What needs to be passed as messages between robots
}

void Robot::init (Pose2D initial_pose) {
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

    if (battery_level < 0.15) {
        return true;
    } else { return false; }
}

/*void resurfaceToCharge() {
    
}*/

resurface to charge (10 "steps" up to surface)

void Robot::move(Pose2D waypoint) {

    // Would be more efficient to just avoid collisions so the white circle never overwrites another robots dot as one moves by
    // As is every robot thread will clear all robots for each of its robots movements

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
    std::cout << "New battery level after move: "  << battery_level << " for robot ID " << id << std::endl;
}

void Robot::updateRobotMessageQueue(Msg msg) {
    std::cout << "IN updateRobotMessageQueue len message_queue: " << message_queue.size() << std::endl;
    std::vector<Msg>& message_queue = getMessageQueue();
    message_queue.push_back(msg);
    std::cout << "END updateRobotMessageQueue len message_queue: " << message_queue.size() << std::endl;

}

void Robot::receiveMessages() { // do we need world as arg or is that redundant?
    std::cout << "in receiveMessages........" << std::endl;
    std::unordered_map<int,std::vector<Msg>>& world_msg_tracker = world->getMessageTracker();
    // Get message from world message tracker by checking receiver index
    int receiverID = getID();
    std::cout << "receiverID: " << receiverID << std::endl;
    std::cout << "Printed msg tracker from world: " << std::endl;
    world->printMessageTracker();
    std::cout << "world msg tracker printed above, size = " << world_msg_tracker.size() << std::endl;
    if (world_msg_tracker.find(receiverID) != world_msg_tracker.end()) { //problem is that we don't ever enter this if statement (start with message tracker print above (nothing prints))
        std::cout << "in if" << std::endl;
        std::vector<Msg>& messages = world_msg_tracker[receiverID]; // Vector of messages queued for current robot
        std::cout << "length of messages from world (should be 1?): " << messages.size() << std::endl;
        std::cout << "!messages.empty(): " << !messages.empty() << std::endl;
        if (!messages.empty()) {
            Msg msg = messages.front();  // Assuming we take the oldest message in queue
            // Add message to receiver robot message queue
            updateRobotMessageQueue(msg);
            std::cout << "Robot " << getID() << " received a message from Robot " << msg.id << std::endl;
        }

    }

}

bool Robot::regroup() {
    // Pull oldest message from robot message queue
    std::cout << " IN REGROUP" << std::endl;
    std::cout << "message_queue length" << message_queue.size() << std::endl;
    if (!message_queue.empty()){
        Msg least_recent_msg = message_queue[0]; // For sake of this test, get message (which rn just has ID)
        std::cout << "MESSAGE EXISTS from robot ID " << least_recent_msg.id << std::endl; 
        //Pose2D goal_pose1{30, 30, 0}; // and if it exists, then trigger robot to go to center (200,200)
        std::cout << "in regroup before true" << std::endl;
        return true;
    }
    std::cout << "in regroup before false" << std::endl;
    return false;
}