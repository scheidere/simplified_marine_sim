#ifndef BEHAVIOR_TREE_HPP
#define BEHAVIOR_TREE_HPP

#include <mutex>

#include "behaviortree_cpp/bt_factory.h"
#include "world.hpp"
#include "robot.hpp"
#include "planners.hpp"
#include "message.hpp"

using namespace BT;


class PlanShortestPath : public SyncActionNode {
private:
	World& _world;
	Robot& _robot;
	ShortestPath& _shortest_path;

public:
	PlanShortestPath(const std::string& name, const BT::NodeConfig& config, World& w, Robot& r, ShortestPath& sp);

    NodeStatus tick() override;

    static PortsList providedPorts();

};

class UseWaypoint : public ThreadedAction {
private:
	World& _world;
	Robot& _robot;

public:
    UseWaypoint(const std::string& name, const NodeConfig& config, World& w, Robot& r);

    NodeStatus tick() override;

    static PortsList providedPorts();
};

class SendMessage : public SyncActionNode {
private:
    World& _world;
    Robot& _sender;

public:
    SendMessage(const std::string& name, const NodeConfig& config, World& world, Robot& sender);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

class ReceiveMessage : public SyncActionNode {
private:
    World& _world;
    Robot& _receiver;

public:
    ReceiveMessage(const std::string& name, const NodeConfig& config, World& world, Robot& receiver);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

#endif