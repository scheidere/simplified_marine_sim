#include <cstdio>
#include "planners.hpp"
#include "scorer.hpp"
#include "robot.hpp"
#include "world.hpp"
#include "rclcpp/rclcpp.hpp"

Robot::Robot(Planner* p, RandomWalkPlanner* rwp, Scorer* s, World* w) : planner(p), random_walk_planner(rwp), scorer(s), world(w) {
    x = 0; y = 0; // Location, currently overwritten in init
}

void Robot::test() {
    std::cout << "Hello world from the robot class!" << std::endl;
}

void Robot::world_test() {
    std::cout << "Testing robot class access to world class (400 = pass): " << world->getX() << std::endl;
}

void Robot::other_tests() {
    std::cout << "Planner: " << planner->getP() << " via robot"<< std::endl;
    std::cout << "Scorer: " << scorer->getS() << " via robot" << std::endl;

}

void Robot::init (int init_x, int init_y, cv::Mat& image) {
    // Red dot at x,y location (what happens if none given for initx inity, will it be default 0,0?)
    x = init_x; y = init_y;
    cv::circle(image, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);

}

void Robot::move (int new_x, int new_y, cv::Mat& image) {
    // Clear old robot location
    cv::circle(image, cv::Point(x, y), 5, cv::Scalar(255, 255, 255), -1);
    // Update x and y
    x = new_x; y = new_y;
    // Add dot at new robot location
    cv::circle(image, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
}

void Robot::followPath(const std::vector<std::pair<int,int>>& path, cv::Mat& image) {
for (const auto& point : path) {
    int new_x = point.first;
    int new_y = point.second;
    //std::cout << new_x << new_y << std::endl;
    move(new_x, new_y, image);
}
}

void Robot::doIteration(cv::Mat& background) {
    int prev_x = getX();
    int prev_y = getY();
    // Need to make planner call on next line actually random walk planner
    std::vector<std::pair<int, int>> step_path = random_walk_planner->randomStep(prev_x, prev_y);
    followPath(step_path, background);
    world->plot(background);
}

RobotController::RobotController(Robot* r) : robot(r) {

}

void RobotController::run (int max_iters, cv::Mat& background) {
    int i = 0;
    std::cout << "max_iters: " << max_iters << std::endl;
    while (rclcpp::ok() && i < max_iters) {
  
      robot->doIteration(background);
      /*okay so this runs for 50 "ticks" and next need to figure out how to show the world and robot movement again within this
      each tick, robot can only move 1 max,
      goal will be to have a multi-pixel path and follow it successfully
      after that, will be learning how to link behavior tree to this*/
      std::cout << "i: " << i << std::endl;
      i++;
    }
}