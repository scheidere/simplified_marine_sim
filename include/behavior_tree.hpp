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
    JSONParser& _parser;

public:
    CheckConvergence(const std::string& name, const NodeConfig& config, World& world, Robot& robot, JSONParser& p);
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

class ExploreB : public ConditionNode {
private:
    Robot& _robot;

public:
    ExploreB(const std::string& name, const NodeConfig& config, Robot& robot, World& world);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

class ExploreC : public ConditionNode {
private:
    Robot& _robot;

public:
    ExploreC(const std::string& name, const NodeConfig& config, Robot& robot, World& world);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

class ExploreD : public ConditionNode {
private:
    Robot& _robot;

public:
    ExploreD(const std::string& name, const NodeConfig& config, Robot& robot, World& world);
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
    std::chrono::high_resolution_clock::time_point _start_time;


public:
    FollowCoveragePath(const std::string& name, const NodeConfig& config, Robot& r, World& w, CoveragePath& cp);

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

    static PortsList providedPorts();
};

class PathClearingNeeded : public ConditionNode {
private:
    Robot& _robot;

public:
    PathClearingNeeded(const std::string& name, const NodeConfig& config, Robot& robot, World& world);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

// old
/*class ClearPath : public StatefulActionNode {
private:
    Robot& _robot;
    World& _world;

public:
    ClearPath(const std::string& name, const NodeConfig& config, Robot& r, World& w);

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

    static PortsList providedPorts();
};*/

// old
/*class CollectSample : public ConditionNode {
private:
    Robot& _robot;

public:
    CollectSample(const std::string& name, const NodeConfig& config, Robot& robot, World& world);
    NodeStatus tick() override;

    static PortsList providedPorts();
};*/

class HandleFailures : public StatefulActionNode {
private:
    World& _world;
    Robot& _robot;

public:
    HandleFailures(const std::string& name, const NodeConfig& config, World& world, Robot& robot);
    //NodeStatus tick() override;
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

    static PortsList providedPorts();
};

class TaskNeededNow : public ConditionNode {
private:
    Robot& _robot;

public:
    TaskNeededNow(const std::string& name, const NodeConfig& config, Robot& robot, World& world);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

class Subtask_1 : public StatefulActionNode {
private:
    Robot& _robot;
    World& _world;
    ShortestPath& _shortest_path_planner;

    std::vector<Pose2D> _waypoints;
    int _current_waypoint_index;

public:
    Subtask_1(const std::string& name, const NodeConfig& config, Robot& r, World& w, ShortestPath& sp);

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

    static PortsList providedPorts();
};

class Subtask_2 : public StatefulActionNode {
private:
    Robot& _robot;
    World& _world;

public:
    Subtask_2(const std::string& name, const NodeConfig& config, Robot& r, World& w);

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

    static PortsList providedPorts();
};

class TestShortPath : public ConditionNode {
private:
    Robot& _robot;

public:
    TestShortPath(const std::string& name, const NodeConfig& config, Robot& robot, World& world);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

// requires fancier action node function to work in single
/*class DoImageArea : public ConditionNode {
private:
    Robot& _robot;

public:
    DoImageArea(const std::string& name, const NodeConfig& config, Robot& robot, World& world);
    NodeStatus tick() override;

    static PortsList providedPorts();
};*/

class DoImageArea1 : public ConditionNode {
private:
    Robot& _robot;

public:
    DoImageArea1(const std::string& name, const NodeConfig& config, Robot& robot, World& world);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

class DoImageArea2 : public ConditionNode {
private:
    Robot& _robot;

public:
    DoImageArea2(const std::string& name, const NodeConfig& config, Robot& robot, World& world);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

class DoImageArea3 : public ConditionNode {
private:
    Robot& _robot;

public:
    DoImageArea3(const std::string& name, const NodeConfig& config, Robot& robot, World& world);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

class DoImageArea4 : public ConditionNode {
private:
    Robot& _robot;

public:
    DoImageArea4(const std::string& name, const NodeConfig& config, Robot& robot, World& world);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

class ImageArea : public StatefulActionNode {
// class ImageArea : public RepeatableStatefulActionNode {
private:
    Robot& _robot;
    World& _world;
    CoveragePath& _coverage_path_planner;

    std::vector<Pose2D> _waypoints;
    int _current_waypoint_index;

public:
    ImageArea(const std::string& name, const NodeConfig& config, Robot& r, World& w, CoveragePath& cp);

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

    static PortsList providedPorts();
};

class DoSampleCollection1 : public ConditionNode {
private:
    Robot& _robot;

public:
    DoSampleCollection1(const std::string& name, const NodeConfig& config, Robot& robot, World& world);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

class DoSampleCollection2 : public ConditionNode {
private:
    Robot& _robot;

public:
    DoSampleCollection2(const std::string& name, const NodeConfig& config, Robot& robot, World& world);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

class DoSampleCollection3 : public ConditionNode {
private:
    Robot& _robot;

public:
    DoSampleCollection3(const std::string& name, const NodeConfig& config, Robot& robot, World& world);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

class DoSampleCollection4 : public ConditionNode {
private:
    Robot& _robot;

public:
    DoSampleCollection4(const std::string& name, const NodeConfig& config, Robot& robot, World& world);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

class DoClearPath1 : public ConditionNode {
private:
    Robot& _robot;

public:
    DoClearPath1(const std::string& name, const NodeConfig& config, Robot& robot, World& world);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

class DoClearPath2 : public ConditionNode {
private:
    Robot& _robot;

public:
    DoClearPath2(const std::string& name, const NodeConfig& config, Robot& robot, World& world);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

/*class CollectSample : public StatefulActionNode {
private:
    Robot& _robot;
    World& _world;
    ShortestPath& _shortest_path_planner;

    std::vector<Pose2D> _waypoints;
    int _current_waypoint_index;

public:
    CollectSample(const std::string& name, const NodeConfig& config, Robot& r, World& w, ShortestPath& sp);

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

    static PortsList providedPorts();
};*/

class ExtractSample : public StatefulActionNode {
private:
    Robot& _robot;
    World& _world;
    ShortestPath& _shortest_path_planner;

    std::vector<Pose2D> _waypoints;
    int _current_waypoint_index;

public:
    ExtractSample(const std::string& name, const NodeConfig& config, Robot& r, World& w, ShortestPath& sp);

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

    static PortsList providedPorts();
};

class LoadSample : public StatefulActionNode {
private:
    Robot& _robot;
    World& _world;
    ShortestPath& _shortest_path_planner;

    std::vector<Pose2D> _waypoints;
    int _current_waypoint_index;

public:
    LoadSample(const std::string& name, const NodeConfig& config, Robot& r, World& w, ShortestPath& sp);

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

    static PortsList providedPorts();
};

class ClearPath : public StatefulActionNode {
private:
    Robot& _robot;
    World& _world;
    ShortestPath& _shortest_path_planner;

    std::vector<Pose2D> _waypoints;
    int _current_waypoint_index;

public:
    ClearPath(const std::string& name, const NodeConfig& config, Robot& r, World& w, ShortestPath& sp);

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

    static PortsList providedPorts();
};

/*class ClearPath : public StatefulActionNode {
private:
    Robot& _robot;
    World& _world;
    ShortestPath& _shortest_path_planner;

    std::vector<Pose2D> _waypoints;
    int _current_waypoint_index;

public:
    ClearPath(const std::string& name, const NodeConfig& config, Robot& r, World& w, ShortestPath& sp);

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

    static PortsList providedPorts();
};*/

/*class ImageArea3 : public StatefulActionNode {
private:
    Robot& _robot;
    World& _world;
    CoveragePath& _coverage_path_planner;

    std::vector<Pose2D> _waypoints;
    int _current_waypoint_index;

public:
    ImageArea3(const std::string& name, const NodeConfig& config, Robot& r, World& w, CoveragePath& cp);

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

    static PortsList providedPorts();
};

class ImageArea4 : public StatefulActionNode {
private:
    Robot& _robot;
    World& _world;
    CoveragePath& _coverage_path_planner;

    std::vector<Pose2D> _waypoints;
    int _current_waypoint_index;

public:
    ImageArea4(const std::string& name, const NodeConfig& config, Robot& r, World& w, CoveragePath& cp);

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

    static PortsList providedPorts();
};*/


class AwayFromHome : public ConditionNode {
private:
    World& _world;
    Robot& _robot;

public:
    AwayFromHome(const std::string& name, const NodeConfig& config, World& world, Robot& robot);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

class IsFailingAlone : public ConditionNode {
private:
    World& _world;
    Robot& _robot;

public:
    IsFailingAlone(const std::string& name, const NodeConfig& config, World& world, Robot& robot);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

class IsStuckWaiting : public ConditionNode {
private:
    World& _world;
    Robot& _robot;

public:
    IsStuckWaiting(const std::string& name, const NodeConfig& config, World& world, Robot& robot);
    NodeStatus tick() override;

    static PortsList providedPorts();
};


class IsIdle : public ConditionNode {
private:
    World& _world;
    Robot& _robot;

public:
    IsIdle(const std::string& name, const NodeConfig& config, World& world, Robot& robot);
    NodeStatus tick() override;

    static PortsList providedPorts();
};

class GoHome : public StatefulActionNode {
private:
    Robot& _robot;
    World& _world;
    ShortestPath& _shortest_path_planner;

    std::vector<Pose2D> _waypoints;
    int _current_waypoint_index;

public:
    GoHome(const std::string& name, const NodeConfig& config, Robot& r, World& w, ShortestPath& sp);

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

    static PortsList providedPorts();
};


#endif