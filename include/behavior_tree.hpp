#ifndef BEHAVIOR_TREE_HPP
#define BEHAVIOR_TREE_HPP

#include <mutex>

#include "behaviortree_cpp/bt_factory.h"
#include "world.hpp"
#include "robot.hpp"
#include "planners.hpp"
#include "message.hpp"
#include "parser.hpp"


using namespace BT;

// Will likely need to update to StatefulActionNode
/*class PlanShortestPath : public SyncActionNode {
private:
	World& _world;
	Robot& _robot;
	ShortestPath& _shortest_path;

public:
	PlanShortestPath(const std::string& name, const BT::NodeConfig& config, World& w, Robot& r, ShortestPath& sp);

    NodeStatus tick() override;

    static PortsList providedPorts();

};*/

// Will likely need to update to StatefulActionNode
/*class PlanCoveragePath : public SyncActionNode {
private:
    World& _world;
    Robot& _robot;
    CoveragePath& _coverage_path;

public:
    PlanCoveragePath(const std::string& name, const BT::NodeConfig& config, World& w, Robot& r, CoveragePath& cp);

    NodeStatus tick() override;

    static PortsList providedPorts();

};

// Will likely need to update to StatefulActionNode
class PlanRegroupPath : public SyncActionNode {
private:
    World& _world;
    Robot& _robot;
    ShortestPath& _shortest_path;

public:
    PlanRegroupPath(const std::string& name, const BT::NodeConfig& config, World& w, Robot& r, ShortestPath& sp);

    NodeStatus tick() override;

    static PortsList providedPorts();

};*/

// Will likely need to update to StatefulActionNode (if used - prob not)
/*class UseWaypoint : public ThreadedAction {
private:
	World& _world;
	Robot& _robot;

public:
    UseWaypoint(const std::string& name, const NodeConfig& config, World& w, Robot& r);

    NodeStatus tick() override;

    static PortsList providedPorts();
};
*/
class Ping : public StatefulActionNode {
private:
    World& _world;
    Robot& _robot;

public:
    Ping(const std::string& name, const NodeConfig& config, World& world, Robot& robot);
    //NodeStatus tick() override;
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

    static PortsList providedPorts();
};


class NewInfoAvailable : public ConditionNode {
private:
    World& _world;
    Robot& _robot;

public:
    NewInfoAvailable(const std::string& name, const NodeConfig& config, World& world, Robot& robot);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

class Communicate : public StatefulActionNode {
private:
    World& _world;
    Robot& _robot;

public:
    Communicate(const std::string& name, const NodeConfig& config, World& world, Robot& robot);
    // NodeStatus tick() override;
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

    static PortsList providedPorts();
};

class NeedRegroup : public ConditionNode {
private:
    Robot& _receiver;

public:
    NeedRegroup(const std::string& name, const NodeConfig& config, Robot& receiver);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

class BuildBundle : public StatefulActionNode {
private:
	Robot& _robot;
    World& _world;
    JSONParser& _parser;

public:
    BuildBundle(const std::string& name, const NodeConfig& config, Robot& r, World& w, JSONParser& p);

    //NodeStatus tick() override;
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

    static PortsList providedPorts();
};

class ResolveConflicts : public StatefulActionNode {
private:
    Robot& _robot;
    World& _world;
    JSONParser& _parser;

public:
    ResolveConflicts(const std::string& name, const NodeConfig& config, Robot& r, World& w, JSONParser& p);

    // NodeStatus tick() override;
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

    static PortsList providedPorts();
};

class CheckConvergence : public ConditionNode {
private:
    World& _world;
    Robot& _robot;

public:
    CheckConvergence(const std::string& name, const NodeConfig& config, World& world, Robot& robot);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

class GreedyTaskAllocator : public StatefulActionNode {
private:
    Robot& _robot;
    World& _world;

public:
    GreedyTaskAllocator(const std::string& name, const NodeConfig& config, Robot& r, World& w);

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

    static PortsList providedPorts();
};

class FollowShortestPath : public StatefulActionNode {
private:
    Robot& _robot;
    World& _world;
    ShortestPath& _shortest_path_planner;

    std::vector<Pose2D> _waypoints;
    int _current_waypoint_index;


public:
    FollowShortestPath(const std::string& name, const NodeConfig& config, Robot& r, World& w, ShortestPath& sp);

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

    static PortsList providedPorts();
};

class ExploreA : public ConditionNode {
private:
    Robot& _robot;

public:
    ExploreA(const std::string& name, const NodeConfig& config, Robot& robot, World& world);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

class FollowCoveragePath : public StatefulActionNode {
private:
    Robot& _robot;
    World& _world;
    CoveragePath& _coverage_path_planner;

    std::vector<Pose2D> _waypoints;
    int _current_waypoint_index;


public:
    FollowCoveragePath(const std::string& name, const NodeConfig& config, Robot& r, World& w, CoveragePath& cp);

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

    static PortsList providedPorts();
};

#endif