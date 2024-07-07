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
    image(init()),
    defineQuadrants()
{

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

void World::clear(Pose2D pose) {
    std::lock_guard<std::mutex> lock(world_mutex);
    std::cout << "Clearing world..." << std::endl;
    cv::circle(image, cv::Point(pose.x, pose.y), 5, cv::Scalar(255, 255, 255), -1);
    // Loop robot tracker
    /*for (auto& pair : robot_tracker) {
        Robot* robot = pair.second;
        cv::circle(image, cv::Point(robot->getPose().x, robot->getPose().y), 5, cv::Scalar(255, 255, 255), -1);
    }*/
}

void World::plot() {
    std::lock_guard<std::mutex> lock(world_mutex); // Lock the mutex
    std::cout << "Plotting world..." << std::endl;
    for (auto& pair : robot_tracker) {
        Robot* robot = pair.second;
        cv::Scalar color = robot->getColor();
        Pose2D robot_pose = robot->getPose();
        std::cout << "Plotting robot ID: " << robot->getID() << " at pose: " << robot_pose.x << ", " << robot_pose.y << " with color: " << color << std::endl;
        cv::circle(image, cv::Point(robot_pose.x, robot_pose.y), 5, color, -1);
    }
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
            std::cout << " Pose: " << ID_robo_pair.second->getPose().x << ", " << ID_robo_pair.second->getPose().y << std::endl;
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

bool World::isCollision(int x, int y) {

    // For now just check for collisions with other robots, but will need to consider obstacles later
    // This is not necessary for the first round of CBBA so leaving this for when we add obstacles to do it all at once
    return {};
}

void World::defineQuadrants() {

    /* -> +x (down +y)
    A   |   B
    ____|____
        | 
    C   |   D
    */
    // Corners:: top left: tl, bottom left: bl, top right: tr, bottom right: br
    Pose2D tlA{0,0,0}; Pose2D blA{0,Y/2,0}; Pose2D trA{X/2,0,0}; Pose2D brA{X/2,Y/2,0};
    areaAcoords.push_back(tlA,blA,trA,brA)
    Pose2D tlB{X/2,0,0}; Pose2D blB{X/2,Y/2,0}; Pose2D trB{X,0,0}; Pose2D brB{X,Y/2,0};
    areaBcoords.push_back(tlB,blB,trB,brB)
    Pose2D tlA{0,Y/2,0}; Pose2D blC{0,Y,0}; Pose2D trC{X/2,0,0}; Pose2D brC{X/2,Y/2,0};
    areaCcoords.push_back(tlC,blC,trC,brC)
    Pose2D tlA{0,0,0}; Pose2D blD{0,Y/2,0}; Pose2D trD{X/2,0,0}; Pose2D brD{X/2,Y/2,0};
    areaDcoords.push_back(tlD,blD,trD,brD)

}

std::vector<Pose2D> World::getQuadrantCenters() {

    std::vector<Pose2D> quadrant_centers;
    Pose2D centerA{X/4,X/4,0};
    Pose2D centerB{X/4,3*X/4,0};
    Pose2D centerC{3*X/4,X/4,0};
    Pose2D centerD{3*X/4,3*X/4,0};
    quadrant_centers.push_back(centerA); // Upper left, area A
    quadrant_centers.push_back(centerB); // Lower left, area B
    quadrant_centers.push_back(centerC); // Upper right, area C
    quadrant_centers.push_back(centerD); // Lower right, area D

    return quadrant_centers;
}