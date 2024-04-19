#ifndef PLANNERS_H
#define PLANNERS_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <random>

class World;
class Robot;

class Planner {
private:
    std::string P;

public:
    Planner();

    void test();

    std::string getP() const { return P; }

    std::vector<std::pair<int, int>> smoothBigStep(int prev_x, int prev_y, int big_x_step, int big_y_step, int num_substeps);

};

class RandomWalkPlanner : public Planner {
private:
      std::mt19937 rng; // Random number generator

public:
    RandomWalkPlanner();

    std::vector<std::pair<int,int>> randomStep(int prev_x, int prev_y);

    void performRandomWalk(cv::Mat& background, int steps, World& world, Robot& robot);


};

#endif
