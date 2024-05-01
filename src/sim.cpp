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

struct Pose2D
{
  double x, y, theta;
};

/*static const char* xml_text = R"(

 <root BTCPP_format="4" >

     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <GenerateWaypoints   name="generate_waypoints"/>
            <PopFromQueue  queue="{waypoints}" popped_item="{wp}" />
            <LoopPose queue="{waypoints}"  value="{wp}">
              <UseWaypoint waypoint="{wp}" />
            </LoopPose>

        </Sequence>
     </BehaviorTree>

 </root>
 )";*/

static const char* xml_text = R"(

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
 )";


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  std::cout << "Running simulation..." << std::endl;

  // Create instances 
  Distance distance; SensorModel sensor_model(&distance); World world (&distance, &sensor_model);
  Planner planner; RandomWalkPlanner random_walk_planner; Scorer scorer; 
  Robot robot(&planner, &random_walk_planner, &scorer, &world); RobotController controller(&robot);

  // Test hierarchy
  /*std::cout << "Testing basic class hierarchy..." << std::endl;
  planner.test();
  scorer.test();
  distance.test();
  sensor_model.test();
  sensor_model.distance_test();
  world.test();
  world.distance_test();
  robot.test();
  robot.world_test();
  robot.other_tests();
  std::cout << "Finished testing basic class hierarchy." << std::endl;*/

  // Initialize world and robot
  cv::Mat background = world.init();
  robot.init(200, 200, background);
  world.plot(background); // Must be after robot init to show robot, do we remove this if it is in controller below?
  
  
  // Max number of iterations
  // int max_iters = 50;
  // controller.run(max_iters, background); // This does not work yet

  // Testing randomwalk
  //int steps = 10;
  //random_walk_planner.performRandomWalk(world, background, robot, steps); // This works!

  // Test BT pipeline
  //RandomWalk rw_node("RW",BT::NodeConfiguration());
  //rw_node.tick();

  // Register RandomWalk node
  BehaviorTreeFactory factory;
  //factory.registerNodeType<RandomWalk>("RandomWalk", std::ref(random_walk_planner),std::ref(world),std::ref(robot),background);
  factory.registerNodeType<GenerateWaypoints>("GenerateWaypoints", std::ref(random_walk_planner),std::ref(world),std::ref(robot),background);
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
