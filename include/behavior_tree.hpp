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

class TestMessages : public SyncActionNode {
private:
    World& _world;
    Robot& _receiver;

public:
    TestMessages(const std::string& name, const NodeConfig& config, World& world, Robot& receiver);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

class Regroup : public ConditionNode {
private:
    Robot& _receiver;

public:
    Regroup(const std::string& name, const NodeConfig& config, Robot& receiver);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

class TestCond : public ConditionNode {
private:
    Robot& _receiver;

public:
    TestCond(const std::string& name, const NodeConfig& config, Robot& receiver);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

class RunTest : public ThreadedAction {
public:
    RunTest(const std::string& name, const NodeConfig& config);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

class RunTest2 : public ThreadedAction {
public:
    RunTest2(const std::string& name, const NodeConfig& config);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

class BuildBundle : public ThreadedAction {
private:
	World& _world;
	Robot& _robot;
	CBBA& _cbba;

public:
    BuildBundle(const std::string& name, const NodeConfig& config, World& w, Robot& r, CBBA& cbba);

    NodeStatus tick() override;

    static PortsList providedPorts();
};

/*class BuildBundle : public ThreadedAction {
private:
    World& _world;
    Robot& _robot;
    CBBA& _cbba;

public:
    BuildBundle(const std::string& name, const NodeConfig& config, World& w, Robot& r, CBBA& cbba);
    NodeStatus tick() override;

    static PortsList providedPorts();
};*/


/*class Test : public SyncActionNode {
private:
    Robot& _robot;
public:
    RunTest(const std::string& name, const NodeConfig& config, Robot& robot);
    NodeStatus tick() override;

    static PortsList providedPorts();
};*/

/*
// Action_A has a different constructor than the default one.
class Action_A: public SyncActionNode
{

public:
    // additional arguments passed to the constructor
    Action_A(const std::string& name, const NodeConfig& config,
             int arg_int, std::string arg_str):
        SyncActionNode(name, config),
        _arg1(arg_int),
        _arg2(arg_str) {}

    // this example doesn't require any port
    static PortsList providedPorts() { return {}; }

    // tick() can access the private members
    NodeStatus tick() override;

private:
    int _arg1;
    std::string _arg2;
};
*/

/*class BuildBundle : public ThreadedAction {
private:
    World& _world;
    Robot& _robot;
    CBBA& _cbba;

public:
    BuildBundle(const std::string& name, const NodeConfig& config, World& w, Robot& r, CBBA& cbba);
    NodeStatus tick() override;

    static PortsList providedPorts();
};*/

/*class BuildBundle : public ThreadedAction {
private:
    World* _world;
    Robot* _robot;
    CBBA* _cbba;

public:
    BuildBundle(const std::string& name, const NodeConfig& config);

    static PortsList providedPorts();

    void setParams(World& world, Robot& robot, CBBA& cbba);

    NodeStatus tick() override;

};
*/
#endif