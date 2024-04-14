#ifndef ROBOT_H
#define ROBOT_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "simulator/world.hpp"

class Robot {
private:
    int x, y;
    World* world; // Point to single,shared world instance

public:
    Robot(World* world);

    void test();
    void world_test();
};

#endif
