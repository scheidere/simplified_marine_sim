#ifndef BEHAVIOR_TREE_HPP
#define BEHAVIOR_TREE_HPP

#include <mutex>

#include "behaviortree_cpp/bt_factory.h"
#include "world.hpp"
#include "robot.hpp"
#include "planners.hpp"

using namespace BT;

class GenerateWaypoints : public SyncActionNode {
private:
	World& _world;
	Robot& _robot;
	cv::Mat _image;

public:
	GenerateWaypoints(const std::string& name, const BT::NodeConfig& config, World& w, Robot& r, cv::Mat image);

    NodeStatus tick() override;

    static PortsList providedPorts();

};

class GenerateNextWaypoint : public SyncActionNode {
private:
	World& _world;
	Robot& _robot;
	cv::Mat _image;

public:
	GenerateNextWaypoint(const std::string& name, const BT::NodeConfig& config, World& w, Robot& r, cv::Mat image);

    NodeStatus tick() override;

    static PortsList providedPorts();

};

class PlanShortestPath : public SyncActionNode {
private:
	World& _world;
	Robot& _robot;
	ShortestPath& _shortest_path;
	cv::Mat& _image;

public:
	PlanShortestPath(const std::string& name, const BT::NodeConfig& config, World& w, Robot& r, ShortestPath& sp, cv::Mat& image);

    NodeStatus tick() override;

    static PortsList providedPorts();

};

class UseWaypoint : public ThreadedAction {
private:
	World& _world;
	Robot& _robot;
	cv::Mat& _image;
	std::mutex& _image_mutex;

public:
    UseWaypoint(const std::string& name, const NodeConfig& config, World& w, Robot& r, cv::Mat& image, std::mutex& image_mutex);

    NodeStatus tick() override;

    static PortsList providedPorts();
};

#endif