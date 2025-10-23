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
    //Pose2D goal_pose; // Not used
    cv::Scalar color;
};

struct TaskInfo {
    int id;
    std::string name;
    std::string type;
    int prerequisite_failures; // only for subtasks
    std::vector<std::string> sub_tasks; // only for main tasks
    int group_size;
    std::unordered_map<std::string, int> group_info;
    std::pair<int,int> location;
    std::unordered_map<std::string,int> area;
    double reward;
};

#endif // STRUCTS_H