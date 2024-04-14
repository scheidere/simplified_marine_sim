#include <cstdio>
#include "simulator/robot.hpp"
#include "simulator/world.hpp"

Robot::Robot(World* w) : world(w) {
}

void Robot::test() {
    std::cout << "Hello world from the robot class!" << std::endl;
}

void Robot::world_test() {
    std::cout << "Testing robot class access to world class: " << world->getX() << std::endl;
}