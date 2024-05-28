#ifndef PLANNERS_H
#define PLANNERS_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <random>
#include "behaviortree_cpp/actions/pop_from_queue.hpp"

class World;
class Robot;

//using matrix = std::vector<std::vector<double>>; // Probably not going to need this
using P = std::pair<double, int>; // (Distance, vertex idx) for priority queue
using adjacency_vector = std::vector<std::vector<P>>;

struct Pose2D
{
  int x, y, theta;
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



    adjacency_vector convertImageToAdjacencyVector(int X, int Y);
    void printAdjacencyVector(const adjacency_vector &adj_vec, int Y);
    //void printVector(std::vector<int> &vector);
    template <typename T>
    void printVector(const std::vector<T> &vec) const;
    std::vector<double> initializeDistances(int V, int robot_start_loc_idx, int Y);
    std::vector<bool> initializeVisits(int V);

    void printPlan(const std::shared_ptr<BT::ProtectedQueue<Pose2D>>& plan);

    int getIndex(int x, int y, int Y);
    std::pair<int,int> getCoords(int idx, int Y);
    bool inBounds(int x, int y, int X, int Y);
    std::vector<P> getNeighbors(int x, int y, int X, int Y);

    double getEuclideanDistance(int x1, int y1, int x2, int y2); // E.g. weight between vertices (pixels) for dijkstra
    
    // Don't need any of the below functions anymore?

    /*matrix initializeDistances(int X, int Y, Pose2D robot_start_loc); //can u use matrix
    matrix initializeVisits(int X, int Y); // can u use matrix

    void printMatrix(std::vector<std::vector<double>> distance_tracker);*/

};

class ShortestPath : public Planner {
public:
    ShortestPath(int step_size);

    // This function generates shortest path to waypoint (does not change sim state)
    std::shared_ptr<BT::ProtectedQueue<Pose2D>> plan(Pose2D current_pose, Pose2D waypoint, int X, int Y);

    // Update distance from start node to current node (given) with respect to its neighbor node
    //std::pair<matrix,matrix> updateDistanceFromStart(Pose2D current, matrix distance_tracker, matrix visit_tracker);


};

#endif
