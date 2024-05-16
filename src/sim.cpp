#include <cstdio>
#include "distance.hpp"
#include "sensor_model.hpp"
#include "world.hpp"
#include "planners.hpp"
#include "scorer.hpp"
#include "robot.hpp"
#include "behavior_tree.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/decorators/loop_node.h"
#include "behaviortree_cpp/actions/pop_from_queue.hpp"


#include <iostream>

using namespace BT;

/*static const char* xml_text = R"(

 <root BTCPP_format="4" >

     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <GenerateWaypoints waypoints="{waypoints}" />
            <QueueSize queue="{waypoints}" size="{wp_size}" />
            <Repeat num_cycles="{wp_size}" >
            <Sequence>
                <PopFromQueue  queue="{waypoints}" popped_item="{wp}" />
                <UseWaypoint waypoint="{wp}" />
            </Sequence>
            </Repeat>
        </Sequence>
     </BehaviorTree>

 </root>
 )";*/
/*
static const char* xml_text = R"(

 <root BTCPP_format="4" >

     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <GenerateNextWaypoint next_waypoint="{next_waypoint}" />
            <QueueSize queue="{next_waypoint}" size="{wp_size}" />
            <Repeat num_cycles="{wp_size}" >
            <Sequence>
                <PopFromQueue  queue="{next_waypoint}" popped_item="{wp}" />
                <UseWaypoint waypoint="{wp}" />
            </Sequence>
            </Repeat>
        </Sequence>
     </BehaviorTree>

 </root>
 )";*/

 static const char* xml_text = R"(

 <root BTCPP_format="4" >

     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <PlanShortestPath path="{path}" />
            <QueueSize queue="{path}" size="{wp_size}" />
            <Repeat num_cycles="{wp_size}" >
            <Sequence>
                <PopFromQueue  queue="{path}" popped_item="{wp}" />
                <UseWaypoint waypoint="{wp}" />
            </Sequence>
            </Repeat>
        </Sequence>
     </BehaviorTree>

 </root>
 )";


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  std::cout << "Running simulation..." << std::endl;

  // How big will the world be (in pixels)?
  const int X = 400; const int Y = 400;

  // Create instances 
  Distance distance; SensorModel sensor_model(&distance); World world (X, Y, &distance, &sensor_model);
  Planner planner; ShortestPath shortest_path; Scorer scorer; 
  Robot robot(&planner, &shortest_path, &scorer, &world);

  std::cout << "World size: " << world.getX() << "x" << world.getY() << std::endl;

  // Initialize world and robot
  cv::Mat background = world.init();
  Pose2D initial_pose{200,200,0}; // Do they have to defined explicitly as doubles?
  robot.init(initial_pose, background);
  world.plot(background); // Must be after robot init to show robot, do we remove this if it is in controller below?

  // Register RandomWalk node
  BehaviorTreeFactory factory;
  //factory.registerNodeType<RandomWalk>("RandomWalk", std::ref(random_walk_planner),std::ref(world),std::ref(robot),background);
  //factory.registerNodeType<GenerateWaypoints>("GenerateWaypoints", std::ref(random_walk_planner),std::ref(world),std::ref(robot),background);
  //factory.registerNodeType<GenerateNextWaypoint>("GenerateNextWaypoint",std::ref(world),std::ref(robot),background);
  factory.registerNodeType<PlanShortestPath>("PlanShortestPath", std::ref(world),std::ref(robot), std::ref(shortest_path), background);
  //factory.registerNodeType<LoopNode<Pose2D>>("LoopPose"); // This results in type errors, changes to deque from  ProtectedQueue
  factory.registerNodeType<QueueSize<Pose2D>>("QueueSize");
  factory.registerNodeType<RepeatNode>("RepeatNode");
  factory.registerNodeType<PopFromQueue<Pose2D>>("PopFromQueue");
  factory.registerNodeType<UseWaypoint>("UseWaypoint", std::ref(world),std::ref(robot),background);

  // Create the behavior tree from the XML text
  auto tree = factory.createTreeFromText(xml_text);

  // Log node statuses (command line)
  StdCoutLogger logger(tree);
  logger.enableTransitionToIdle(false);

  // Execute the behavior tree
  tree.tickWhileRunning();
  
  std::cout << "Terminate by pressing any key..." << std::endl;
  cv::waitKey(0);

  return 0;
}
