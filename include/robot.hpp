#ifndef ROBOT_H
#define ROBOT_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include "planners.hpp"
#include "scorer.hpp"
#include "world.hpp"
//#include "message.hpp"
#include "structs.hpp"
#include "CBBA.hpp"

struct Msg;
//struct Task;
//struct Bundle;

class Robot {
private:
    int id;
    int task_id;
    Pose2D pose;
    Pose2D goal;
    Planner* planner;
    ShortestPath* shortest_path;
    CoveragePath* coverage_path;
    Scorer* scorer;
    World* world; // Point to single,shared world instance
    std::vector<Msg> message_queue;
    cv::Scalar color;
    double battery_level;
    std::vector<Task> assignable_tasks;
    /*WinningBids winning_bids(world);
    WinningAgentIndices winning_agent_indices(world);*/
    //WinningBids winning_bids; // y_i; One double for each task j, initialized with zeros
    //WinningAgentIndices winning_agent_indices; // z_i; One for each task j, so if 2 at index j in this vector, that means agent 2 has highest bid on task j
    Bundle bundle; // Empty initially
    Path path; // Empty initially
    WinningBids winning_bids;
    WinningAgentIndices winning_agent_indices;
    void initializeWinningBidsAndIndices();
    std::ofstream robot_log; // Init file for each robot to log in



public:
    Robot(Planner* planner, ShortestPath* shortest_path, CoveragePath* coverage_path, Scorer* scorer, World* world, const Pose2D& initial_pose, const Pose2D& goal_pose, std::vector<Task> tasks, int robot_id, cv::Scalar color); // Is this right with planners?

    int getID() const { return id; }
    int getCurrentTaskID() const { return task_id; }
    int getX() const { return pose.x; }
    int getY() const { return pose.y; }
    Pose2D getPose() const { return pose; }
    Pose2D getGoalPose() const { return goal; }
    cv::Scalar getColor() const {return color; }
    Bundle& getBundle() { return bundle; }
    Path& getPath() { return path; }
    void init(Pose2D initial_pose);
    void printTasksVector();
    void move(Pose2D waypoint);
    std::vector<Msg>& getMessageQueue() { return message_queue; }
    void printMessageQueue(const std::vector<Msg>&  message_queue);
    void printMessage(Msg msg);
    void updateRobotMessageQueue(Msg msg);
    void receiveMessages();
    bool needRegroup();
    double getBatteryLevel() const { return battery_level; }
    void updateBatteryLevel(double drain_percent);
    bool batteryLow();
    WinningBids& getWinningBids() { return winning_bids; }
    WinningAgentIndices& getWinningAgentIndices() { return winning_agent_indices; }
    std::string generateLogFilename();
    void log(std::string log_msg);

    //void resurfaceToCharge();

    // Will need to add a function/functions that deal with battery level checking and eval wrt tasks
    // Like how do we know, given a task, how much battery it could take, do we translate distance to battery?
    // If we are given a four corner defined area, then do we calculate distance to traverse the entire thing, and translate that to battery?
};

#endif
