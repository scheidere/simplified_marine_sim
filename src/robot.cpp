#include <cstdio>
#include "planners.hpp"
#include "scorer.hpp"
#include "robot.hpp"
#include "world.hpp"
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/actions/pop_from_queue.hpp"


// Create state class, which will contain robot condition functions

Robot::Robot(Planner* p, ShortestPath* sp, Scorer* s, World* w, const Pose2D& goal_pose, int robot_id) 
: planner(p), shortest_path(sp), scorer(s), world(w), goal(goal_pose), id(robot_id) {
    pose = {0, 0, 0}; // Is this used? 
    goal = goal_pose; // Like return to home or drop off item loc, specific to each robot
    id = robot_id; // Robot ID
    world->trackRobot(this); //id,this pair or just this?


    // Add current task ID
    // Where will we record things like at_place, in_comms, battery_low, etc.?
    // What needs to be passed as messages between robots
}

void Robot::test() {
    std::cout << "Hello world from the robot class!" << std::endl;
}

void Robot::world_test() {
    std::cout << "Testing robot class access to world class (400 = pass): " << world->getX() << std::endl;
}

void Robot::other_tests() {
    //std::cout << "Planner: " << planner->getP() << " via robot"<< std::endl;
    std::cout << "Scorer: " << scorer->getS() << " via robot" << std::endl;

}

void Robot::init (Pose2D initial_pose) {
    // Access world for image
    cv::Mat image = world->getImage();
    // Red dot at x,y location
    cv::circle(image, cv::Point(initial_pose.x, initial_pose.y), 5, cv::Scalar(0, 0, 255), -1);
    pose = initial_pose; // Set current robot pose variable

}

void Robot::move(Pose2D waypoint) {
    // Access world for image
    cv::Mat image = world->getImage();

    // Clear old robot location
    cv::circle(image, cv::Point(pose.x, pose.y), 5, cv::Scalar(255, 255, 255), -1);
    // Update sim image to reflect robot movement
    cv::circle(image, cv::Point(waypoint.x, waypoint.y), 5, cv::Scalar(0, 0, 255), -1);
    world->plot(); // Added to work with BT
    pose = waypoint; // Update x and y within robot class
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