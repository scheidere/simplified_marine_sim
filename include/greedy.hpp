#ifndef GREEDY_H
#define GREEDY_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <cmath>
#include <limits>
#include "message.hpp"
#include "planners.hpp"
#include "distance.hpp"
#include "world.hpp"
#include "robot.hpp"
#include "structs.hpp"
#include "parser.hpp"


struct Msg;


class Greedy {
private:

    Robot& robot; // Reference to specific robot on team
    World& world;

public:
    Greedy(Robot& robot, World& world);

    void run();
    double getTaskScore(int task_id, int prev_x, int prev_y);

};

#endif