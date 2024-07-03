#ifndef ROBOT_H
#define ROBOT_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "planners.hpp"
#include "scorer.hpp"
#include "world.hpp"
#include "message.hpp"
#include "CBBA.hpp"

struct Msg;

class Robot {
private:
    int id;
    int task_id;
    Pose2D pose;
    Pose2D goal;
    Planner* planner;
    ShortestPath* shortest_path;
    Scorer* scorer;
    World* world; // Point to single,shared world instance
    std::vector<Msg> message_queue;
    cv::Scalar color;
    Bundle bundle; // Empty initially
    double battery_level;

public:
    Robot(Planner* planner, ShortestPath* shortest_path, Scorer* scorer, World* world, const Pose2D& goal_pose, int robot_id, cv::Scalar color); // Is this right with planners?

    int getID() const { return id; }
    int getCurrentTaskID() const { return task_id; }
    int getX() const { return pose.x; }
    int getY() const { return pose.y; }
    Pose2D getPose() const { return pose; }
    Pose2D getGoalPose() const { return goal; }
    cv::Scalar getColor() const {return color; }
    Bundle getBundle() const { return bundle; }
    void init(Pose2D initial_pose);
    void move(Pose2D waypoint);
    std::vector<Msg>& getMessageQueue() { return message_queue; }
    void updateRobotMessageQueue(Msg msg);
    void receiveMessages();
    bool regroup();
    double getBatteryLevel() const { return battery_level; }
    void updateBatteryLevel(double drain_percent);

    // Will need to add a function/functions that deal with battery level checking and eval wrt tasks
    // Like how do we know, given a task, how much battery it could take, do we translate distance to battery?
    // If we are given a four corner defined area, then do we calculate distance to traverse the entire thing, and translate that to battery?
};

#endif
