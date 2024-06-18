#ifndef ROBOT_H
#define ROBOT_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "planners.hpp"
#include "scorer.hpp"
#include "world.hpp"
#include "message.hpp"

struct Msg;

class Robot {
private:
    int id;
    Pose2D pose;
    Pose2D goal;
    Planner* planner;
    ShortestPath* shortest_path;
    Scorer* scorer;
    World* world; // Point to single,shared world instance
    std::vector<Msg> message_queue;

public:
    Robot(Planner* planner, ShortestPath* shortest_path, Scorer* scorer, World* world, const Pose2D& goal_pose, int robot_id); // Is this right with planners?

    int getID() const { return id; }
    int getX() const { return pose.x; }
    int getY() const { return pose.y; }
    Pose2D getPose() const { return pose; }
    Pose2D getGoalPose() const { return goal; }
    void test();
    void world_test();
    void other_tests();
    void init(Pose2D initial_pose);
    void move(Pose2D waypoint);
    std::vector<Msg> getMessageQueue() { return message_queue; }
    void updateRobotMessageQueue(Msg msg);
    void receiveMessages();
    void regroup();
};

#endif
