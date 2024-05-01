#ifndef BEHAVIOR_TREE_HPP
#define BEHAVIOR_TREE_HPP

#include "behaviortree_cpp/bt_factory.h"
#include "world.hpp"
#include "robot.hpp"
#include "planners.hpp"

using namespace BT;

class RandomWalk : public SyncActionNode {
private:
	RandomWalkPlanner& _rwp; // Try pointer * next
	World& _world;
	Robot& _robot;
	cv::Mat _background;

public:
	RandomWalk(const std::string& name, const BT::NodeConfig& config, RandomWalkPlanner& rwp, World& w, Robot& r, cv::Mat background);

    NodeStatus tick() override;

    // A node having ports MUST implement this STATIC method
    static PortsList providedPorts();

};

class GenerateWaypoints : public SyncActionNode {
private:
	RandomWalkPlanner& _rwp; // Try pointer * next
	World& _world;
	Robot& _robot;
	cv::Mat _background;

public:
	GenerateWaypoints(const std::string& name, const BT::NodeConfig& config, RandomWalkPlanner& rwp, World& w, Robot& r, cv::Mat background);

    NodeStatus tick() override;

    static PortsList providedPorts();

};

class UseWaypoint : public ThreadedAction {
private:
	World& _world;
	Robot& _robot;
	cv::Mat _background;

public:
    UseWaypoint(const std::string& name, const NodeConfig& config, World& w, Robot& r, cv::Mat background);

    NodeStatus tick() override;

    static PortsList providedPorts();
};

#endif