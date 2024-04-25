#include <cstdio>
#include "distance.hpp"
#include "sensor_model.hpp"
#include "world.hpp"
#include "planners.hpp"
#include "scorer.hpp"
#include "robot.hpp"
#include "behavior_tree.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include <iostream>

using namespace BT;

static const char* xml_text = R"(

 <root BTCPP_format="4" >

     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <RandomWalk   name="random_walk"/>
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
  factory.registerNodeType<RandomWalk>("RandomWalk", std::ref(random_walk_planner),std::ref(world),std::ref(robot),background);
  //factory.registerNodeType<RandomWalk>("RandomWalk");

  // Create the behavior tree from the XML text
  auto tree = factory.createTreeFromText(xml_text);

  // Execute the behavior tree
  tree.tickWhileRunning();
  
  std::cout << "Terminate by pressing any key..." << std::endl;
  cv::waitKey(0);

  return 0;
}
