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
private:
    std::shared_ptr<BT::ProtectedQueue<Pose2D>> current_plan;
    double step_size = 5; // In pixels

public:
    Planner();

    void test();

    // Getter/setter functions for private variables
    std::shared_ptr<BT::ProtectedQueue<Pose2D>> getPlan() const { return current_plan; }
    double getStepSize() const { return step_size; }
    void updatePlan(std::shared_ptr<BT::ProtectedQueue<Pose2D>> new_plan) { current_plan = new_plan; }
    void updateStepSize(double new_step_size) { step_size = new_step_size; }    

    double euclideanDistance(cv::Point p1, cv::Point p2); // E.g. weight between vertices (pixels) for dijkstra

    template <int N, int M>
    void printDistancesArray(std::array<std::array<double, N>, M>& arr);

    // Given map, return matrix with distances each pixel are (initially) from starting point (0 at point and inf elsewhere)
    void initializeDistances(cv::Mat image, cv::Point p1);

    //std::vector<std::pair<int, int>> smoothBigStep(int prev_x, int prev_y, int big_x_step, int big_y_step, int num_substeps);

};

class ShortestPath : public Planner {
public:
    ShortestPath();

    // This function generates shortest path to waypoint (does not change sim state)
    std::shared_ptr<BT::ProtectedQueue<Pose2D>> plan(Pose2D current_pose, Pose2D waypoint);


};

#endif
