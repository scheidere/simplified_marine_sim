#ifndef ROBOT_H
#define ROBOT_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "planners.hpp"
#include "scorer.hpp"
#include "world.hpp"

class Robot {
private:
    Pose2D pose;
    Planner* planner;
    ShortestPath* shortest_path; // Is this needed? Replaced randomwalk
    Scorer* scorer;
    World* world; // Point to single,shared world instance

public:
    Robot(Planner* planner, ShortestPath* shortest_path, Scorer* scorer, World* world); // Is this right with planners?

    double getX() const { return pose.x; }
    double getY() const { return pose.y; }
    Pose2D getPose() const { return pose; }
    void test();
    void world_test();
    void other_tests();
    void init(Pose2D initial_pose, cv::Mat& image);
    //void doIteration(cv::Mat& image);
    void move(Pose2D waypoint, cv::Mat& image);
    void followPath(std::shared_ptr<BT::ProtectedQueue<Pose2D>> path, cv::Mat& image);
};

#endif
