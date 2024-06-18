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

/*using namespace BT;

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
            <SendMessage sender="{sender_robot}" />
            <ReceiveMessage receiver="{receiver_robot}" />
        </Sequence>
     </BehaviorTree>

 </root>
 )";*/

static const char* xml_text = R"(

<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <PlanShortestPath path="{path}" />
            <QueueSize queue="{path}" size="{wp_size}" />
            <Repeat num_cycles="{wp_size}">
                <Sequence>
                    <PopFromQueue queue="{path}" popped_item="{wp}" />
                    <UseWaypoint waypoint="{wp}" />
                </Sequence>
            </Repeat>
            <SendMessage waypoint="{wp}"/>
            <ReceiveMessage waypoint="{wp}"/>
        </Sequence>
     </BehaviorTree>
</root>
)";

void run_robot(int robot_id, Pose2D initial_pose, Pose2D goal_pose, int step_size, Planner& planner, ShortestPath& shortest_path, Scorer& scorer, World& world) {

    // Should we also change colors of each robot or add id number to visual?

    std::cout << "Creating robot " << robot_id << " with step size " << step_size << "..." << std::endl;

    // Initialize robot
    Robot robot(&planner, &shortest_path, &scorer, &world, goal_pose, robot_id);
    {
        robot.init(initial_pose);
        world.plot();
        std::cout << "ID Check in Robot " << robot.getID() << std::endl;
    } // Mutex unlocks

    // Register BT nodes using std::ref to ensure actual class instaces are used and not copies
    BehaviorTreeFactory factory;
    factory.registerNodeType<PlanShortestPath>("PlanShortestPath", std::ref(world), std::ref(robot), std::ref(shortest_path));
    factory.registerNodeType<QueueSize<Pose2D>>("QueueSize");
    factory.registerNodeType<RepeatNode>("RepeatNode");
    factory.registerNodeType<PopFromQueue<Pose2D>>("PopFromQueue");
    factory.registerNodeType<UseWaypoint>("UseWaypoint", std::ref(world), std::ref(robot));
    factory.registerNodeType<SendMessage>("SendMessage", std::ref(world), std::ref(robot));
    factory.registerNodeType<ReceiveMessage>("ReceiveMessage", std::ref(world), std::ref(robot));

    // Create behavior tree
    auto tree = factory.createTreeFromText(xml_text); // See MainTree XML above

    // Log node statuses (command line)
    //StdCoutLogger logger(tree); // THIS IS AN ISSUE, might need mutex for this too?
    //logger.enableTransitionToIdle(false);

    // Execute the behavior tree
    tree.tickWhileRunning();

    std::cout << "Robot " << robot_id << " simulation complete." << std::endl;

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
  const double comms_range = 50.0; //

  Distance distance; SensorModel sensor_model(&distance); World world (X, Y, &distance, &sensor_model, comms_range);
  Planner planner (step_size); ShortestPath shortest_path (step_size); Scorer scorer;

  std::cout << "Inits are done..." << std::endl;

  // Example way to pass different goals to shortest path in each robot
  Pose2D initial_pose1{20, 10, 0};
  Pose2D initial_pose2{10, 10, 0};
  Pose2D goal_pose1{20, 7, 0}; // Robot 1 goal
  Pose2D goal_pose2{10, 7, 0}; // Robot 2 goal

  // Create and start threads for each robot
  std::thread robot1(run_robot, 1, initial_pose1, goal_pose1, step_size, std::ref(planner), std::ref(shortest_path), std::ref(scorer), std::ref(world));
  std::thread robot2(run_robot, 2, initial_pose2, goal_pose2, step_size, std::ref(planner), std::ref(shortest_path), std::ref(scorer), std::ref(world));

  std::cout << "Threads started..." << std::endl;

  // Wait for both threads to finish
  robot1.join();
  robot2.join();

  std::cout << "Both threads finished" << std::endl;

  std::cout << "Terminate by pressing any key..." << std::endl;
  cv::waitKey(0);

  return 0;
}
