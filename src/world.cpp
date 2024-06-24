#include <cstdio>
#include "distance.hpp"
#include "sensor_model.hpp"
#include "world.hpp"
#include "robot.hpp"

World::World(int X, int Y, Distance* d, SensorModel* s, double comms_range) 
    : X(X),
    Y(Y), 
    distance(d), 
    sensor_model(s),
    comms_range(comms_range),
    image(init())
{
    //cv::Mat image = init(); // Create world background image of dimensions X and Y
}

cv::Mat& World::getImage() { 
    std::lock_guard<std::mutex> lock(world_mutex);
    return image; 

}

cv::Mat World::init() {
    // White background, mutex not necessary for init
    std::cout << "Initializing world..." << std::endl;
    cv::Mat image(X, Y, CV_8UC3, cv::Scalar(255, 255, 255));

    return image;
}

// Like other functions with mutex, should only be used outside this class
std::unordered_map<int, Robot*>& World::getRobotTracker() { 
    std::lock_guard<std::mutex> lock(world_mutex);
    //std::cout << "IN GETROBOTTRACKER" << std::endl;
    return robot_tracker;
}

// Like other functions with mutex, should only be used outside this class
std::unordered_map<int,std::vector<Msg>>& World::getMessageTracker() {
    std::lock_guard<std::mutex> lock(world_mutex);
    return message_tracker;
}

void World::plot() {
    // No mutex because only used in world functions that are already locked
    std::cout << "Plotting world..." << std::endl;
    cv::imshow("Quadrant Image", image);
    cv::waitKey(300);
}

void World::trackRobot(Robot* robot) { // Save robot instances by ID int
    std::lock_guard<std::mutex> lock(world_mutex);
    //std::cout << "IN TRACKROBOT; Tracking Robot ID: " << robot->getID() << std::endl;
    robot_tracker[robot->getID()] = robot;
}

void World::printTrackedRobots() {
    std::lock_guard<std::mutex> lock(world_mutex);
    std::cout << "Tracked Robots:" << std::endl;
    for (const auto& ID_robo_pair : robot_tracker) {
        std::cout << " Raw Robot ID: " << ID_robo_pair.first << std::endl;
        if (ID_robo_pair.second != nullptr) {
            std::cout << " From Instance Robot ID: " << ID_robo_pair.second->getID() << std::endl;
        } else {
            std::cout << " Null Robot instance" << std::endl;
        }
    }
}

void World::printMessageTracker() {
    std::lock_guard<std::mutex> lock(world_mutex);
    std::cout << "World message tracker:" << std::endl;
    //std::cout << message_tracker.size() << std::endl;
    for (auto& pair : message_tracker) {
        int receiverID = pair.first;
        std::vector<Msg>& messages = pair.second;

        std::cout << "Receiver ID: " << receiverID << std::endl;
        for (auto& msg : messages) {
            std::cout << "  From ID: " << msg.id << std::endl;
        }
    }
    std::cout << "in print msg tracker END" << std::endl;
}

bool World::inComms(int id1, int id2) {
    std::lock_guard<std::mutex> lock(world_mutex);
    //std::cout << "in inComms func after mutex" << std::endl;
    if (robot_tracker.find(id1) == robot_tracker.end() || robot_tracker.find(id2) == robot_tracker.end()) {
        //std::cout << "One or both robots does not exist! Cannot check if they are close enough for comms." << std::endl;
        return false;
    }
    Robot* robot1 = robot_tracker[id1];
    Robot* robot2 = robot_tracker[id2];
    Pose2D p1 = robot1->getPose(); Pose2D p2 = robot2->getPose();
    double distance_between_robots = distance->getEuclideanDistance(p1.x,p1.y,p2.x,p2.y);
    return distance_between_robots <= comms_range;
}