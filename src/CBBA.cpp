#include <cstdio>
#include <cmath>
#include "CBBA.hpp"
#include "robot.hpp"



CBBA::CBBA() {

}

double CBBA::createBid(Robot * robot, Task& task) {
    // Based on utility and robot state

    // Utility already considers distance and priority, so now we consider battery level
    double battery_weight = 1; // What should this weight be? Maybe it weighs more when battery level is beyond certain thresholds?
    double battery_level = robot->getBatteryLevel();
    if (battery_level > 0.5) {
        battery_weight = 0.1; // Less concerned about battery when it is above 50%
    } else if (battery_level > 0.1) {
        battery_weight = 0.5;
    } else { // < 10% battery level
        battery_weight = 1;
    }

    // High battery level, the higher the bid (so inversed and higher battery weight for lower battery level)
    return task.utility - battery_weight * (1.0/battery_level);

}

double CBBA::calculateUtility(Robot * robot, Task& task) {
    // Based on task aspects, e.g., how easy, how important

    // Importance weights
    double distance_weight = 1; // for now it is 1, since we are not considering anything else
    double priority_weight = 1; // Not sure how we want to weigh priority vs distance so leaving 1 as well

    // Task info
    Pose2D robot_location = robot->getPose(); // Current robot location
    Pose2D task_location = task.location; // Center if there is a whole area to explore, otherwise singular task location, like a drop off point for an item
    double distance = Distance::getEuclideanDistance(robot_location.x,robot_location.y,task_location.x,task_location.y);
    double task_priority = task.priority;

    // We want higher utility value for lower distance, because that means easier
    // and we also want higher utility for higher task priority
    return distance_weight * (1.0/distance) + priority_weight * task_priority;


}

// Fancy functions here
void CBBA::buildBundle(Robot * robot) {

}

void CBBA::printBundle() {

}


void CBBA::obtainConsensus() {

}


