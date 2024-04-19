#ifndef ROBOT_H
#define ROBOT_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "planners.hpp"
#include "scorer.hpp"
#include "world.hpp"

class Robot {
private:
    int x, y;
    Planner* planner;
    RandomWalkPlanner* random_walk_planner;
    Scorer* scorer;
    World* world; // Point to single,shared world instance

public:
    Robot(Planner* planner, RandomWalkPlanner* random_walk_planner, Scorer* scorer, World* world);

    void test();
    void world_test();
    void other_tests();
    void init(int init_x, int init_y, cv::Mat& image);
    int getX() const { return x; }
    int getY() const { return y; }
    void doIteration(cv::Mat& image);
    void move(int new_x, int new_y, cv::Mat& image);
    void followPath(const std::vector<std::pair<int,int>>& path, cv::Mat& image);
};


class RobotController {
private:
    Robot* robot;

public:
    RobotController(Robot* r);

    void run(int max_iters, cv::Mat& background);
};

#endif
