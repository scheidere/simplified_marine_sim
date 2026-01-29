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
    World* world;
    Robot* robot;

public:
    Planner(int step_size, World* w, Robot* robot);

    void test();

    // Getter/setter functions for private variables
    std::shared_ptr<BT::ProtectedQueue<Pose2D>> getPlan() const { return current_plan; }
    int getStepSize() const { return step_size; }
    //void updatePlan(std::shared_ptr<BT::ProtectedQueue<Pose2D>> new_plan) { current_plan = new_plan; }
    void updateStepSize(double new_step_size) { step_size = new_step_size; }   

    adjacency_vector convertImageToAdjacencyVector(int X, int Y);
    void printAdjacencyVector(const adjacency_vector &adj_vec, int Y);
    template <typename T>
    void printVector(const std::vector<T> &vec) const;
    std::vector<double> initializeDistances(int V, int robot_start_loc_idx, int Y);
    std::vector<bool> initializeVisits(int V);

    //void printPlan(const std::shared_ptr<BT::ProtectedQueue<Pose2D>>& plan); // deprecated

    int getIndex(int x, int y, int Y);
    std::pair<int,int> getCoords(int idx, int Y);
    bool inBounds(int x, int y, int X, int Y);
    std::vector<P> getNeighbors(int x, int y, int X, int Y); // added world for isObstacle call

};

class ShortestPath : public Planner {
public:
    ShortestPath(int step_size, World* w, Robot* r);

    // This function generates shortest path to waypoint (does not change sim state)
    //std::shared_ptr<BT::ProtectedQueue<Pose2D>> plan(Pose2D current_pose, Pose2D waypoint, int X, int Y); // This was original way
    std::vector<Pose2D> plan(Pose2D current_pose, Pose2D waypoint, int X, int Y); // new way, for single FollowShortestPath call in BT


};

class CoveragePath : public ShortestPath {
protected:
    int obs_radius; // Observation radius in pixels

public:
    CoveragePath(int step_size, int obs_radius, World* w, Robot* r);

    int getObsRadius() const { return obs_radius; }

    // This function generates shortest path to waypoint (does not change sim state)

    std::vector<Pose2D> generateBoustrophedonWaypoints(std::unordered_map<std::string,int> area);
    std::vector<Pose2D> plan(Pose2D current_pose, std::unordered_map<std::string,int> area, int X, int Y); // new way, for single FollowCoveragePath call in BT


};

#endif
