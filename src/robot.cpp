#include <cstdio>
#include "planners.hpp"
#include "scorer.hpp"
#include "robot.hpp"
#include "world.hpp"
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/actions/pop_from_queue.hpp"

Robot::Robot(Planner* p, ShortestPath* sp, Scorer* s, World* w) : planner(p), shortest_path(sp), scorer(s), world(w) {
    pose = {0, 0, 0};
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

void Robot::init (Pose2D initial_pose, cv::Mat& image) {
    // Red dot at x,y location
    pose = initial_pose;
    cv::circle(image, cv::Point(pose.x, pose.y), 5, cv::Scalar(0, 0, 255), -1);

}

void Robot::move(Pose2D waypoint, cv::Mat& image) {
    // Clear old robot location
    cv::circle(image, cv::Point(pose.x, pose.y), 5, cv::Scalar(255, 255, 255), -1);
    // Update sim image to reflect robot movement
    cv::circle(image, cv::Point(pose.x, pose.y), 5, cv::Scalar(0, 0, 255), -1);
    world->plot(image); // Added to work with BT
    // Update x and y within robot class
    pose = waypoint;
}
