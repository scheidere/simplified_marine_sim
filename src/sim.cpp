#include <cstdio>
#include <mutex>
#include <thread>
#include <exception>
#include <iostream>
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
#include "parser.hpp"

// Run to show robots traversing to different waypoints
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
)";*/

// Run to test message broadcasting and receipt
// RunTest helps prints organization
// Issue is that broadcast only happens once and receive only happens once (not per robot, so only one sends, and one receives instead of both)
// Adding shortestpath block beneath sequence before the message testing, in case issue is initialization timing
/*static const char* xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <SendMessage/>
            <PlanShortestPath path="{path}" goal="{goal}" />
            <QueueSize queue="{path}" size="{wp_size}" />
            <Repeat num_cycles="{wp_size}">
                <Sequence>
                    <PopFromQueue queue="{path}" popped_item="{wp}" />
                    <UseWaypoint waypoint="{wp}" />
                </Sequence>
            </Repeat>
            <ReceiveMessage/>
            <NeedRegroup/>
            <RunTest waypoint="{wp}"/> 
        </Sequence>
     </BehaviorTree>
</root>
)";*/

// Test path traversal triggered by a condition, which relates to robots sending and having received messages
// First let's test a delay function (threaded action so more than one tick, and just have it wait for 5 seconds between send and receive)
// HERE NEXT
/*static const char* xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <SendMessage/>
            <PlanShortestPath path="{path}" goal="{goal}" />
            <QueueSize queue="{path}" size="{wp_size}" />
            <Repeat num_cycles="{wp_size}">
                <Sequence>
                    <PopFromQueue queue="{path}" popped_item="{wp}" />
                    <UseWaypoint waypoint="{wp}" />
                </Sequence>
            </Repeat>
            <ReceiveMessage/>
            <NeedRegroup/>
            <RunTest waypoint="{wp}"/> 
        </Sequence>
     </BehaviorTree>
</root>
)";*/

/*static const char* xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <SendMessage/>
            <ReceiveMessage/>
            <Sequence name="regroup_sequence">
                <NeedRegroup/>
                <PlanRegroupPath path="{path}" />
                <QueueSize queue="{path}" size="{wp_size}" />
                <Repeat num_cycles="{wp_size}">
                <Sequence>
                    <PopFromQueue queue="{path}" popped_item="{wp}" />
                    <UseWaypoint waypoint="{wp}" />
                </Sequence>
            </Repeat>
            </Sequence>
        </Sequence>
     </BehaviorTree>
</root>
)";*/

/*static const char* xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <PlanCoveragePath path="{path}" />
            <QueueSize queue="{path}" size="{wp_size}" />
            <Repeat num_cycles="{wp_size}">
            <Sequence>
                <PopFromQueue queue="{path}" popped_item="{wp}" />
                <UseWaypoint waypoint="{wp}" />
            </Sequence>
            </Repeat>
        </Sequence>
     </BehaviorTree>
</root>
)";*/

// <RunTest waypoint="{wp}"/> 

// Run to test BuildBundle only (currently crashes)
static const char* xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <BuildBundle/>
        </Sequence>
     </BehaviorTree>
</root>
)";

double getCurrentTime() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() / 1000.0;
}


void run_robot(int robot_id, Pose2D initial_pose, Pose2D goal_pose, std::vector<Task> assignable_tasks, cv::Scalar color, int step_size, Planner& planner, ShortestPath& shortest_path, CoveragePath& coverage_path, Scorer& scorer, World& world, JSONParser& parser) {
    std::cout << "Entering run_robot for robot " << robot_id << std::endl;

    try {
        std::cout << "Creating robot " << robot_id << " with step size " << step_size << "..." << std::endl;

        try {
            Robot robot(&planner, &shortest_path, &coverage_path, &scorer, &world, &parser, initial_pose, goal_pose, assignable_tasks, robot_id, color);
            std::cout << "Robot " << robot_id << " created successfully." << std::endl;

            {
                world.plot();
                std::cout << "ID Check in Robot " << robot.getID() << std::endl;
                robot.printTasksVector();
            }

            // Register Behavior Tree nodes
            std::cout << "Registering Behavior Tree nodes for robot " << robot_id << "..." << std::endl;
            BehaviorTreeFactory factory;

            try {
                factory.registerNodeType<PlanShortestPath>("PlanShortestPath", std::ref(world), std::ref(robot), std::ref(shortest_path));
                factory.registerNodeType<PlanCoveragePath>("PlanCoveragePath", std::ref(world), std::ref(robot), std::ref(coverage_path));
                factory.registerNodeType<PlanRegroupPath>("PlanRegroupPath", std::ref(world), std::ref(robot), std::ref(shortest_path));
                factory.registerNodeType<QueueSize<Pose2D>>("QueueSize");
                factory.registerNodeType<RepeatNode>("RepeatNode");
                factory.registerNodeType<PopFromQueue<Pose2D>>("PopFromQueue");
                factory.registerNodeType<UseWaypoint>("UseWaypoint", std::ref(world), std::ref(robot));
                factory.registerNodeType<SendMessage>("SendMessage", std::ref(world), std::ref(robot));
                factory.registerNodeType<ReceiveMessage>("ReceiveMessage", std::ref(world), std::ref(robot));
                factory.registerNodeType<TestMessages>("TestMessages", std::ref(world), std::ref(robot));
                factory.registerNodeType<NeedRegroup>("NeedRegroup", std::ref(robot));
                factory.registerNodeType<TestCond>("TestCond", std::ref(robot));
                factory.registerNodeType<RunTest>("RunTest");
                factory.registerNodeType<RunTest2>("RunTest2");
                factory.registerNodeType<BuildBundle>("BuildBundle", std::ref(robot)); // Threaded action with args
                //factory.registerNodeType<Test>("Test", std::ref(robot));
                //factory.registerNodeType<RunTest>("BuildBundle", std::ref(world), std::ref(robot), std::ref(cbba));
                /*factory.registerNodeType<BuildBundle>("BuildBundle", [&](const std::string& name, const BT::NodeConfig& config) {
                    auto node = std::make_shared<BuildBundle>(name, config);
                    node->setParams(world, robot, cbba);
                    return node;
                });*/

                std::cout << "Behavior Tree nodes registered successfully for robot " << robot_id << "." << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "Exception caught during BehaviorTreeFactory node registration for robot " << robot_id << ": " << e.what() << std::endl;
                throw;
            }

            try {
                std::cout << "Creating behavior tree for robot " << robot_id << "..." << std::endl;
                auto tree = factory.createTreeFromText(xml_text);
                std::cout << "Behavior tree created successfully for robot " << robot_id << "." << std::endl;

                std::cout << "Starting tree tick for robot " << robot_id << "..." << std::endl;
                tree.tickWhileRunning();
                std::cout << "Tree tick completed for robot " << robot_id << "." << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "Exception caught during behavior tree creation or execution for robot " << robot_id << ": " << e.what() << std::endl;
                throw;
            }

            std::cout << "Robot " << robot_id << " simulation complete." << std::endl;
        } catch (const std::length_error& e) {
            std::cerr << "std::length_error caught during robot creation or initialization for robot " << robot_id << ": " << e.what() << std::endl;
            throw;
        } catch (const std::exception& e) {
            std::cerr << "Exception caught during robot creation or initialization for robot " << robot_id << ": " << e.what() << std::endl;
            throw;
        }
    } catch (const std::length_error& e) {
        std::cerr << "std::length_error caught in run_robot for robot " << robot_id << ": " << e.what() << std::endl;
        throw;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in run_robot for robot " << robot_id << ": " << e.what() << std::endl;
        throw;
    } catch (...) {
        std::cerr << "Unknown exception caught in run_robot for robot " << robot_id << std::endl;
        throw;
    }

    std::cout << "Exiting run_robot for robot " << robot_id << std::endl;
}

int main(int argc, char** argv) {
    try {
        double start_time = getCurrentTime();
        std::cout << "Running simulation..." << std::endl;

        // Testing parsing
        //std::string path = std::filesystem::current_path().append("src/simplified_marine_sim/config/input.json");
        //std::string path = (std::filesystem::current_path() / "src/simplified_marine_sim/config/input.json").string(); also works
        //std::cout << path << std::endl;
        //JSONParser parser(path);
        //parser.parseJSON(path);

        const int X = 400;
        const int Y = 400;
        const int step_size = 1;
        if (step_size != 1) {
            std::cout << "Your step size is larger than 1! Have you made changes in planners for smaller final step/neighbors etc.?" << std::endl;
            std::cin.get();
        }
        const double comms_range = 50.0;
        const int obs_radius = 4;

        std::string path = std::filesystem::current_path().append("src/simplified_marine_sim/config/input.json");
        JSONParser parser(path);

        Distance distance;
        SensorModel sensor_model(&distance);
        World world(X, Y, &distance, &sensor_model, &parser, comms_range);
        Planner planner(step_size, obs_radius);
        ShortestPath shortest_path(step_size);
        CoveragePath coverage_path(step_size, obs_radius);
        Scorer scorer;

        std::cout << "Inits are done..." << std::endl;
        std::cout << "************** Testing CBBA stuff **************" << std::endl;

        std::vector<Task> allTasks;
        try {
            allTasks = world.getAllTasks();
        } catch (const std::exception& e) {
            std::cerr << "Exception caught while getting tasks from world: " << e.what() << std::endl;
            return -1;
        }

        std::vector<Task> assignable_tasks1;
        std::vector<Task> assignable_tasks2;
        try {
            assignable_tasks1 = { allTasks[0], allTasks[1], allTasks[2], allTasks[3] };
            assignable_tasks2 = { allTasks[0], allTasks[1], allTasks[2], allTasks[3] };
        } catch (const std::out_of_range& e) {
            std::cerr << "std::out_of_range caught while accessing tasks: " << e.what() << std::endl;
            return -1;
        }

        std::vector<cv::Scalar> colors = {
            cv::Scalar(255, 0, 0),
            cv::Scalar(0, 255, 0),
            cv::Scalar(0, 0, 255),
            cv::Scalar(0, 255, 255),
            cv::Scalar(255, 255, 0),
            cv::Scalar(255, 0, 255),
            cv::Scalar(255, 255, 255)
        };

        Pose2D initial_pose1{0, 0, 0};
        Pose2D initial_pose2{20, 10, 0};
        Pose2D goal_pose1{10, 10, 0};
        Pose2D goal_pose2{5, 10, 0};
        cv::Scalar color1 = cv::Scalar(0, 0, 255);
        cv::Scalar color2 = cv::Scalar(255, 0, 0);

        try {
            std::thread robot1(run_robot, 1, initial_pose1, goal_pose1, assignable_tasks1, color1, step_size, std::ref(planner), std::ref(shortest_path), std::ref(coverage_path), std::ref(scorer), std::ref(world), std::ref(parser));
            std::thread robot2(run_robot, 2, initial_pose2, goal_pose2, assignable_tasks2, color2, step_size, std::ref(planner), std::ref(shortest_path), std::ref(coverage_path), std::ref(scorer), std::ref(world), std::ref(parser));

            std::cout << "Threads started..." << std::endl;

            robot1.join();
            robot2.join();
        } catch (const std::exception& e) {
            std::cerr << "Exception caught while starting or joining threads: " << e.what() << std::endl;
            return -1;
        }

        std::cout << "Both threads finished" << std::endl;

        double end_time = getCurrentTime();
        double total_time = end_time - start_time;
        std::cout << "Execution time: " << total_time << std::endl;

        std::cout << "Terminate by pressing any key..." << std::endl;
        cv::waitKey(0);

        return 0;
    } catch (const std::length_error& e) {
        std::cerr << "std::length_error caught in main: " << e.what() << std::endl;
        return -1;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in main: " << e.what() << std::endl;
        return -1;
    }
}