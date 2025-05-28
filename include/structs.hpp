#ifndef STRUCTS_H
#define STRUCTS_H

#include <vector>
#include <stdexcept>
#include <iostream>
#include <limits>
#include <string>


struct AgentInfo {
    int id;
    std::string type;
    Pose2D initial_pose;
    Pose2D goal_pose;
    cv::Scalar color;
};

struct TaskInfo {
    int id;
    std::string type;
    std::pair<int,int> location;
    std::unordered_map<std::string,int> area;
    double reward;
};

#endif // STRUCTS_H