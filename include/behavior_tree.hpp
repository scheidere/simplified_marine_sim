#ifndef BEHAVIOR_TREE_HPP
#define BEHAVIOR_TREE_HPP

#include "behaviortree_cpp/bt_factory.h"
#include "world.hpp"
#include "robot.hpp"
#include "planners.hpp"

using namespace BT;

class GenerateWaypoints : public SyncActionNode {
private:
	World& _world;
	Robot& _robot;
	cv::Mat _background;

public:
	GenerateWaypoints(const std::string& name, const BT::NodeConfig& config, World& w, Robot& r, cv::Mat background);

    NodeStatus tick() override;

    static PortsList providedPorts();

};

class GenerateNextWaypoint : public SyncActionNode {
private:
	World& _world;
	Robot& _robot;
	cv::Mat _background;

public:
	GenerateNextWaypoint(const std::string& name, const BT::NodeConfig& config, World& w, Robot& r, cv::Mat background);

    NodeStatus tick() override;

    static PortsList providedPorts();

};

class PlanShortestPath : public SyncActionNode {
private:
	World& _world;
	Robot& _robot;
	ShortestPath& _shortest_path;
	cv::Mat _background;

public:
	PlanShortestPath(const std::string& name, const BT::NodeConfig& config, World& w, Robot& r, ShortestPath& sp, cv::Mat background);

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