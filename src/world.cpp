#include <cstdio>
#include "distance.hpp"
#include "sensor_model.hpp"
#include "world.hpp"
#include "robot.hpp"

struct Task;

World::World(int X, int Y, Distance* d, SensorModel* s, double comms_range) 
    : X(X),
    Y(Y), 
    distance(d), 
    sensor_model(s),
    comms_range(comms_range),
    image(init())
{
    try {
        defineQuadrants();
        initAllTasks();
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in World constructor: " << e.what() << std::endl;
        throw; // Re-throw to propagate the exception
    }
}

cv::Mat& World::getImage() { 
    std::lock_guard<std::mutex> lock(world_mutex);
    return image; 
}

cv::Mat World::init() {
    try {
        std::cout << "Initializing world..." << std::endl;
        cv::Mat image(X, Y, CV_8UC3, cv::Scalar(255, 255, 255));
        return image;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in init: " << e.what() << std::endl;
        throw; // Re-throw to propagate the exception
    }
}

std::unordered_map<int, Robot*>& World::getRobotTracker() { 
    std::lock_guard<std::mutex> lock(world_mutex);
    return robot_tracker;
}

std::unordered_map<int,std::vector<Msg>>& World::getMessageTracker() {
    std::lock_guard<std::mutex> lock(world_mutex);
    return message_tracker;
}

std::vector<Task>& World::getAllTasks() {
    std::lock_guard<std::mutex> lock(world_mutex);
    std::cout << "Returning all tasks. Number of tasks: " << allTasks.size() << std::endl;
    return allTasks;
}

int World::getTaskIndex(Task task_j) {
    std::lock_guard<std::mutex> lock(world_mutex);

    auto it = std::find_if(allTasks.begin(), allTasks.end(), [&](const Task& task) {
        return task.id == task_j.id;
    });

    int j = std::distance(allTasks.begin(), it);

    return j;
}

void World::clear(Pose2D pose) {
    std::lock_guard<std::mutex> lock(world_mutex);
    std::cout << "Clearing world..." << std::endl;
    cv::circle(image, cv::Point(pose.x, pose.y), 5, cv::Scalar(255, 255, 255), -1);
}

void World::plot() {
    std::lock_guard<std::mutex> lock(world_mutex);
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

void World::trackRobot(Robot* robot) {
    std::lock_guard<std::mutex> lock(world_mutex);
    std::cout << "Tracking Robot ID: " << robot->getID() << std::endl;
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
    for (auto& pair : message_tracker) {
        int receiverID = pair.first;
        std::vector<Msg>& messages = pair.second;

        std::cout << "Receiver ID: " << receiverID << std::endl;
        std::cout << "Number of messages for receiver: " << messages.size() << std::endl;
        for (auto& msg : messages) {
            //std::cout << "  From ID: " << msg.id << std::endl;
            std::cout << "Printing message..." << std::endl;
            printMessage(msg);
        }
    }
    std::cout << "End of printMessageTracker" << std::endl;
}

void World::printMessage(Msg msg) { // no mutex because used within the function above
    std::cout << "Message ID:" << msg.id << "\n";
    std::cout << "Task ID: " << msg.task_id << "\n";
    std::cout << "Location: (" << msg.location.x << ", " << msg.location.y << ", " << msg.location.theta << ")\n";
   /* std::cout << "Bundle: [";
    for (const auto& task : msg.bundle.tasks) {
        std::cout << task << " "; // Assuming tasks can be printed this way
    }
    std::cout << "]\n";*/
}

bool World::inComms(int id1, int id2) {
    std::lock_guard<std::mutex> lock(world_mutex);
    if (robot_tracker.find(id1) == robot_tracker.end() || robot_tracker.find(id2) == robot_tracker.end()) {
        return false;
    }
    Robot* robot1 = robot_tracker[id1];
    Robot* robot2 = robot_tracker[id2];
    Pose2D p1 = robot1->getPose(); Pose2D p2 = robot2->getPose();
    double distance_between_robots = distance->getEuclideanDistance(p1.x,p1.y,p2.x,p2.y);
    return distance_between_robots <= comms_range;
}

bool World::isCollision(int x, int y) {
    return {};
}

void World::defineQuadrants() {
    try {
        Pose2D tlA{0,0,0}; Pose2D blA{0,Y/2,0}; Pose2D trA{X/2,0,0}; Pose2D brA{X/2,Y/2,0};
        areaACoords.push_back(tlA); areaACoords.push_back(blA); areaACoords.push_back(trA); areaACoords.push_back(brA);
        Pose2D tlB{0,Y/2,0}; Pose2D blB{0,Y,0}; Pose2D trB{X/2,Y/2,0}; Pose2D brB{X/2,Y,0};
        areaBCoords.push_back(tlB); areaBCoords.push_back(blB); areaBCoords.push_back(trB); areaBCoords.push_back(brB);
        Pose2D tlC{X/2,0,0}; Pose2D blC{X/2,Y/2,0}; Pose2D trC{X,0,0}; Pose2D brC{X,Y/2,0};
        areaCCoords.push_back(tlC); areaCCoords.push_back(blC); areaCCoords.push_back(trC); areaCCoords.push_back(brC);
        Pose2D tlD{X/2,Y/2,0}; Pose2D blD{X/2,Y,0}; Pose2D trD{X,Y/2,0}; Pose2D brD{X,Y,0};
        areaDCoords.push_back(tlD); areaDCoords.push_back(blD); areaDCoords.push_back(trD); areaDCoords.push_back(brD);

        std::cout << "Quadrants defined." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in defineQuadrants: " << e.what() << std::endl;
        throw; // Re-throw to propagate the exception
    }
}

std::vector<Pose2D> World::getQuadrantCenters() {
    std::vector<Pose2D> quadrant_centers;
    try {
        Pose2D centerA{X/4,X/4,0};
        Pose2D centerB{X/4,3*X/4,0};
        Pose2D centerC{3*X/4,X/4,0};
        Pose2D centerD{3*X/4,3*X/4,0};
        quadrant_centers.push_back(centerA);
        quadrant_centers.push_back(centerB);
        quadrant_centers.push_back(centerC);
        quadrant_centers.push_back(centerD);

        int size = 5;

        cv::rectangle(image, cv::Point(centerA.x - size, centerA.y - size), cv::Point(centerA.x + size, centerA.y + size), cv::Scalar(0, 0, 0), -1);
        cv::rectangle(image, cv::Point(centerB.x - size, centerB.y - size), cv::Point(centerB.x + size, centerB.y + size), cv::Scalar(0, 0, 0), -1);
        cv::rectangle(image, cv::Point(centerC.x - size, centerC.y - size), cv::Point(centerC.x + size, centerC.y + size), cv::Scalar(0, 0, 0), -1);
        cv::rectangle(image, cv::Point(centerD.x - size, centerD.y - size), cv::Point(centerD.x + size, centerD.y + size), cv::Scalar(0, 0, 0), -1);

        std::cout << "Quadrant centers defined: " << quadrant_centers.size() << " centers." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Exception caught in getQuadrantCenters: " << e.what() << std::endl;
        throw; // Re-throw to propagate the exception
    }
    return quadrant_centers;
}

void World::initAllTasks() {
    try {
        std::vector<Pose2D> quadrant_centers = getQuadrantCenters();

        if (quadrant_centers.size() < 4) {
            throw std::runtime_error("Not enough quadrant centers to initialize tasks.");
        }

        Task exploreA(1, "Explore area A", quadrant_centers[0], 0, 0, 0); 
        allTasks.push_back(exploreA);
        Task exploreB(2, "Explore area B", quadrant_centers[1], 0, 0, 0); 
        allTasks.push_back(exploreB);
        Task exploreC(3, "Explore area C", quadrant_centers[2], 0, 0, 0); 
        allTasks.push_back(exploreC);
        Task exploreD(4, "Explore area D", quadrant_centers[3], 0, 0, 0); 
        allTasks.push_back(exploreD);

        std::cout << "Initialized all tasks. Number of tasks: " << allTasks.size() << std::endl;

        // Check the size of allTasks
        if (allTasks.size() > allTasks.max_size()) {
            throw std::length_error("The number of tasks exceeds the maximum allowable size.");
        }
    } catch (const std::length_error& e) {
        std::cerr << "std::length_error caught in initAllTasks: " << e.what() << std::endl;
        throw; // Re-throw to propagate the exception
    } catch (const std::runtime_error& e) {
        std::cerr << "std::runtime_error caught in initAllTasks: " << e.what() << std::endl;
        throw; // Re-throw to propagate the exception
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in initAllTasks: " << e.what() << std::endl;
        throw; // Re-throw to propagate the exception
    }
}