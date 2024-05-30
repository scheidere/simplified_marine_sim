#include <cstdio>
#include <mutex>
#include <thread>
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

void run_robot(int id, Pose2D initial_pose, int step_size, Planner& planner, ShortestPath& shortest_path, Scorer& scorer, World& world, cv::Mat& image, std::mutex& image_mutex) {

    // Should we also change colors of each robot or add id number to visual?

    std::cout << "Creating robot " << id << " with step size " << step_size << "..." << std::endl;

    // Initialize robot
    Robot robot(&planner, &shortest_path, &scorer, &world);
    { // Protect shared image with mutex
        std::lock_guard<std::mutex> lock(image_mutex);
        robot.init(initial_pose, image, image_mutex);
        world.plot(image);
    } // Mutex unlocks

    // Register BT nodes using std::ref to ensure actual class instaces are used and not copies
    BehaviorTreeFactory factory;
    factory.registerNodeType<PlanShortestPath>("PlanShortestPath", std::ref(world), std::ref(robot), std::ref(shortest_path), std::ref(image));
    factory.registerNodeType<QueueSize<Pose2D>>("QueueSize");
    factory.registerNodeType<RepeatNode>("RepeatNode");
    factory.registerNodeType<PopFromQueue<Pose2D>>("PopFromQueue");
    factory.registerNodeType<UseWaypoint>("UseWaypoint", std::ref(world), std::ref(robot), std::ref(image), std::ref(image_mutex));

    // Create behavior tree
    auto tree = factory.createTreeFromText(xml_text); // See MainTree XML above

    // Log node statuses (command line)
    StdCoutLogger logger(tree);
    logger.enableTransitionToIdle(false);

    // Execute the behavior tree
    tree.tickWhileRunning();

    std::cout << "Robot " << id << " simulation complete." << std::endl;

 }


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  std::cout << "Running simulation..." << std::endl;


  // ## Common Initialization ## //
  // How big will the world be (in pixels)?
  const int X = 400; // Down
  const int Y = 400; // Right
  // How far can the robot(s) go in one step? LEAVE AS 1 unless you double check usage, esp. in planners.
  const int step_size = 1; // In pixels
  if (step_size != 1) {
    std::cout << "Your step size is larger than 1! Have made changes in planners for smaller final step/neighbors etc.?" << std::endl;
    std::cin.get();
  }

  Distance distance; SensorModel sensor_model(&distance); World world (X, Y, &distance, &sensor_model);
  Planner planner (step_size); ShortestPath shortest_path (step_size); Scorer scorer; // (step_size) for shortest path removed for test EMILY

  // Initialize world
  cv::Mat image = world.getImage(); // shared resource
  std::mutex image_mutex; // mutex object
  Pose2D initial_pose1{0, 0, 0};
  Pose2D initial_pose2{10, 10, 0};

  std::cout << "Inits are done..." << std::endl;

  // Why pass both image and image mutex? Why not just use image mutex for all?
  // Will want to pass different goals to shortest path in each robot
  // 

  // Create and start threads for each robot
  std::thread robot1(run_robot, 1, initial_pose1, step_size, std::ref(planner), std::ref(shortest_path), std::ref(scorer), std::ref(world), std::ref(image), std::ref(image_mutex));
  std::thread robot2(run_robot, 2, initial_pose2, step_size, std::ref(planner), std::ref(shortest_path), std::ref(scorer), std::ref(world), std::ref(image), std::ref(image_mutex));

  std::cout << "Threads started..." << std::endl;

  // Wait for both threads to finish
  robot1.join();
  robot2.join();

  std::cout << "Both threads finished" << std::endl;

  std::cout << "Terminate by pressing any key..." << std::endl;
  cv::waitKey(0);

  return 0;
}
