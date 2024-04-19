#include <cstdio>
#include "distance.hpp"
#include "sensor_model.hpp"
#include "world.hpp"
#include "planners.hpp"
#include "scorer.hpp"
#include "robot.hpp"


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
  std::cout << "Testing basic class hierarchy..." << std::endl;
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
  std::cout << "Finished testing basic class hierarchy." << std::endl;

  // Initialize world and robot
  cv::Mat background = world.init();
  robot.init(200, 200, background);
  world.plot(background); // Must be after robot init to show robot, do we remove this if it is in controller below?

  // Max number of iterations
  // int max_iters = 50;
  // controller.run(max_iters, background); // This does not work yet

  // Testing randomwalk
  int steps = 10;
  random_walk_planner.performRandomWalk(background, steps, world, robot); // This works!
  // I think there are issues with having a smoothed step occur in one iteration as it is currently written
  // Probably other issues too, I think I had just started with the controller when the alienware broke
  // With the above random walk executing, you cant terminate by pressing any key, you instead close the world window
  // Next step will be to trigger random walk but from the robot controller, make sure iterations are working
  // Don't dwell on random walk appearance, move on after iterations working and robot moving
  // Behavior tree triggering of this is next step after that

  std::cout << "Terminate by pressing any key..." << std::endl;
  cv::waitKey(0);

  return 0;
}
