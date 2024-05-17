#ifndef PLANNERS_H
#define PLANNERS_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <random>
#include "behaviortree_cpp/actions/pop_from_queue.hpp"

class World;
class Robot;

struct Pose2D
{
  double x, y, theta;
};


class Planner {
protected:
    std::shared_ptr<BT::ProtectedQueue<Pose2D>> current_plan;
    int step_size; // In pixels

public:
    Planner(int step_size);

    void test();

    // Getter/setter functions for private variables
    std::shared_ptr<BT::ProtectedQueue<Pose2D>> getPlan() const { return current_plan; }
    int getStepSize() const { return step_size; }
    void updatePlan(std::shared_ptr<BT::ProtectedQueue<Pose2D>> new_plan) { current_plan = new_plan; }
    void updateStepSize(double new_step_size) { step_size = new_step_size; }    

    double euclideanDistance(cv::Point p1, cv::Point p2); // E.g. weight between vertices (pixels) for dijkstra

    std::vector<std::vector<double>> initializeDistances(int X, int Y, Pose2D robot_start_loc);

    void printDistances(std::vector<std::vector<double>> distance_array);

};

class ShortestPath : public Planner {
public:
    ShortestPath(int step_size);

    // This function generates shortest path to waypoint (does not change sim state)
    std::shared_ptr<BT::ProtectedQueue<Pose2D>> plan(Pose2D current_pose, Pose2D waypoint);

    // Update distance from start node to current node (given) with respect to its neighbor node
    //double updateDistanceFromStart(Pose2D current, Pose2D neighbor);


};

#endif
