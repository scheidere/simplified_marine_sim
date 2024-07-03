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
#include "CBBA.hpp"



#include <iostream>

// This tree sends both robots to their goal locations, then they send each other messages and regroup to the same location if they hear them
// For now, that location is set at 0,0 as opposed to a location splitting the distance between a robot and another
/*static const char* xml_text = R"(

<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <PlanShortestPath path="{path}" goal="{goal}" />
            <QueueSize queue="{path}" size="{wp_size}" />
            <Repeat num_cycles="{wp_size}">
                <Sequence>
                    <PopFromQueue queue="{path}" popped_item="{wp}" />
                    <UseWaypoint waypoint="{wp}" />
                </Sequence>
            </Repeat>
            <SendMessage waypoint="{wp}"/>
            <ReceiveMessage waypoint="{wp}"/>
            <Sequence>
              <Regroup rendezvous="{rv}"/>
              <PlanShortestPath path="{path2}" goal="{rv}" />
              <QueueSize queue="{path2}" size="{wp_size}" />
              <Repeat num_cycles="{wp_size}">
                  <Sequence>
                      <PopFromQueue queue="{path2}" popped_item="{wp2}" />
                      <UseWaypoint waypoint="{wp2}" />
                  </Sequence>
              </Repeat>
              </Sequence>
        </Sequence>
     </BehaviorTree>
</root>
)";*/

// This tree is to test a moving robot passing by a stationary one
/*static const char* xml_text = R"(

<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <PlanShortestPath path="{path}" goal="{goal}" />
            <QueueSize queue="{path}" size="{wp_size}" />
            <Repeat num_cycles="{wp_size}">
                <Sequence>
                    <PopFromQueue queue="{path}" popped_item="{wp}" />
                    <UseWaypoint waypoint="{wp}" />
                </Sequence>
            </Repeat>
            <RunTest waypoint="{wp}"/>
        </Sequence>
     </BehaviorTree>
</root>
)";
*/

// This tree is for testing initial bundle assembly only
static const char* xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <Bundle />
        </Sequence>
     </BehaviorTree>
</root>
)";


void run_robot(int robot_id, Pose2D initial_pose, Pose2D goal_pose, cv::Scalar color, int step_size, Planner& planner, ShortestPath& shortest_path, Scorer& scorer, World& world) {

    // Should we also change colors of each robot or add id number to visual?

    std::cout << "Creating robot " << robot_id << " with step size " << step_size << "..." << std::endl;

    // Initialize robot
    Robot robot(&planner, &shortest_path, &scorer, &world, goal_pose, robot_id, color);
    {
        robot.init(initial_pose);
        world.plot();
        std::cout << "ID Check in Robot " << robot.getID() << std::endl;
    } // Mutex unlocks

    // Register BT nodes using std::ref to ensure actual class instaces are used and not copies
    BehaviorTreeFactory factory;
    //factory.registerNodeType<Parallel>("Parallel");
    factory.registerNodeType<PlanShortestPath>("PlanShortestPath", std::ref(world), std::ref(robot), std::ref(shortest_path));
    factory.registerNodeType<QueueSize<Pose2D>>("QueueSize");
    factory.registerNodeType<RepeatNode>("RepeatNode");
    factory.registerNodeType<PopFromQueue<Pose2D>>("PopFromQueue");
    factory.registerNodeType<UseWaypoint>("UseWaypoint", std::ref(world), std::ref(robot));
    factory.registerNodeType<SendMessage>("SendMessage", std::ref(world), std::ref(robot));
    factory.registerNodeType<ReceiveMessage>("ReceiveMessage", std::ref(world), std::ref(robot));
    factory.registerNodeType<Regroup>("Regroup", std::ref(robot));
    factory.registerNodeType<TestCond>("TestCond", std::ref(robot));
    factory.registerNodeType<RunTest>("RunTest"); // sync action node cannot return running

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
  Planner planner (step_size); ShortestPath shortest_path (step_size); Scorer scorer; CBBA cbba;

  std::cout << "Inits are done..." << std::endl;

  std::cout << "************** Testing CBBA stuff **************" << std::endl;

  // Define a Pose2D object
  Pose2D task_location{1, 2, 0};
  Pose2D task_location2{10, 20, 0};

  //std::vector<Pose2D> locations = {taskLocation};
  //std::vector<Pose2D> locations2 = {taskLocation2};

  // Create a Task object (id, name, loc, priority, utility, bid)
  Task task1(1, "Deliver Package", task_location, 1.0, 50.0, 1.0);
  Task task2(2, "Deliver Package 2", task_location2, 2.0, 75.0, 2.0);

  Bundle bundle;
  bundle.addTask(task1);
  bundle.addTask(task2);

  bundle.print();

  Task nexttask = bundle.popNextTask();

  nexttask.print();

  std::cout << "Press Enter to continue..." << std::endl;
  std::cin.get();

  // Robot colors
  std::vector<cv::Scalar> colors = {
      cv::Scalar(255, 0, 0),    // Blue
      cv::Scalar(0, 255, 0),    // Green
      cv::Scalar(0, 0, 255),    // Red
      cv::Scalar(0, 255, 255),  // Yellow
      cv::Scalar(255, 255, 0),  // Cyan
      cv::Scalar(255, 0, 255),  // Magenta
      cv::Scalar(255, 255, 255) // White
  };

  // Example way to pass different goals to shortest path in each robot
  Pose2D initial_pose1{10, 10, 0};
  Pose2D initial_pose2{20, 10, 0};
  //Pose2D goal_pose1{10, 7, 0}; 
  //Pose2D goal_pose2{20, 7, 0};
  Pose2D goal_pose1{10,10,0};
  Pose2D goal_pose2{5,10,0};
  cv::Scalar color1 = cv::Scalar(0, 0, 255); // Red color
  cv::Scalar color2 = cv::Scalar(255, 0, 0); // Blue color

  // Create and start threads for each robot
  std::thread robot1(run_robot, 1, initial_pose1, goal_pose1, color1, step_size, std::ref(planner), std::ref(shortest_path), std::ref(scorer), std::ref(world));
  std::thread robot2(run_robot, 2, initial_pose2, goal_pose2, color2, step_size, std::ref(planner), std::ref(shortest_path), std::ref(scorer), std::ref(world));

  std::cout << "Threads started..." << std::endl;

  // Wait for both threads to finish
  robot1.join();
  robot2.join();

  std::cout << "Both threads finished" << std::endl;

  std::cout << "Terminate by pressing any key..." << std::endl;
  cv::waitKey(0);

  return 0;
}
