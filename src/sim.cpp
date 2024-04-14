#include <cstdio>
#include "simulator/sensor_model.hpp"
#include "simulator/world.hpp"
#include "simulator/robot.hpp"


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("Running simulation...\n");

  // Create instances 
  Distance distance; SensorModel sensor_model(&distance); World world; Robot robot(&world);

  // Test hierarchy
  distance.test();
  sensor_model.test();
  sensor_model.distance_test();
  world.test();
  robot.test();
  robot.world_test();

  return 0;
}
