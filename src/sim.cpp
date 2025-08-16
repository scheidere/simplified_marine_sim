#include <cstdio>
#include <mutex>
#include <thread>
#include <exception>
#include <iostream>
#include "distance.hpp"
#include "sensor_model.hpp"
#include "world.hpp"
#include "planners.hpp"
#include "robot.hpp"
#include "behavior_tree.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/decorators/loop_node.h"
#include "behaviortree_cpp/actions/pop_from_queue.hpp"
#include "CBBA.hpp"
#include "parser.hpp"
#include "utils.hpp"


// Current best tree (CBBA only)
/*static const char* xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <ParallelAll max_failures="2">
            <Repeat num_cycles="-1">
            <Ping/>
            </Repeat>
            <RepeatSequence name="unlimited_repeat">
                <NewInfoAvailable/>
                <RepeatSequence name="threshold_repeat" convergence_threshold="5" cumulative_convergence_count_in="{ccc}" cumulative_convergence_count_out="{ccc}">
                    <BuildBundle/>
                    <Communicate/>
                    <ResolveConflicts/>
                    <CheckConvergence cumulative_convergence_count_in="{ccc}" cumulative_convergence_count_out="{ccc}" />
                </RepeatSequence>
            </RepeatSequence>
        </ParallelAll>
     </BehaviorTree>
</root>
)";*/

// Greedy version (Sequence instead of parallel root node because no replanning during execution) [Seems to work as expected! Creates path of tasks]
/*static const char* xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <GreedyTaskAllocator/>
        </Sequence>
     </BehaviorTree>
</root>
)";*/

// Now testing local task execution subtree (worked but now deprecated because port info passing removed for flexibility (logic added to functions directly))
/*static const char* xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <GreedyTaskAllocator/>
            <ExploreA start_loc = "{stl}" />
            <FollowShortestPath goal_loc = "{stl}" />
        </Sequence>
     </BehaviorTree>
</root>
)";*/

/*static const char* xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <GreedyTaskAllocator/>
            <ExploreA/>
            <FollowCoveragePath/>
            <ExploreB/>
        </Sequence>
     </BehaviorTree>
</root>
)";
*/

// Working greedy and exploration tree
/*static const char* xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <ParallelAll max_failures="3">
            <GreedyTaskAllocator/>
            <RepeatSequence>
                <ExploreA/>
                <FollowCoveragePath/>
            </RepeatSequence>
            <RepeatSequence>
                <ExploreB/>
                <FollowCoveragePath/>
            </RepeatSequence>
            <RepeatSequence>
                <ExploreC/>
                <FollowCoveragePath/>
            </RepeatSequence>
            <RepeatSequence>
                <ExploreD/>
                <FollowCoveragePath/>
            </RepeatSequence>
        </ParallelAll>
     </BehaviorTree>
</root>
)";*/

//NOTE: IF YOU CHANGE CONVERGENCE THRESHOLD, IT MUST MATCH input.json
/*static const char* xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <ParallelAll max_failures="6">
            <Repeat num_cycles="-1">
            <Ping/>
            </Repeat>
            <RepeatSequence name="unlimited_repeat">
                <NewInfoAvailable/>
                <RepeatSequence name="threshold_repeat" convergence_threshold="5" cumulative_convergence_count_in="{ccc}" cumulative_convergence_count_out="{ccc}">
                    <BuildBundle/>
                    <Communicate/>
                    <ResolveConflicts/>
                    <CheckConvergence cumulative_convergence_count_in="{ccc}" cumulative_convergence_count_out="{ccc}" />
                </RepeatSequence>
            </RepeatSequence>
            <RepeatSequence>
                <ExploreA/>
                <FollowCoveragePath/>
            </RepeatSequence>
            <RepeatSequence>
                <ExploreB/>
                <FollowCoveragePath/>
            </RepeatSequence>
            <RepeatSequence>
                <ExploreC/>
                <FollowCoveragePath/>
            </RepeatSequence>
            <RepeatSequence>
                <ExploreD/>
                <FollowCoveragePath/>
            </RepeatSequence>
        </ParallelAll>
     </BehaviorTree>
</root>
)";*/

// For CBGA testing
/*static const char* xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <ParallelAll max_failures="2">
            <Repeat num_cycles="-1">
            <Ping/>
            <Communicate/>
        </ParallelAll>
     </BehaviorTree>
</root>
)";*/

// Testing updateLocations()
static const char* xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <ParallelAll max_failures="2">
            <FollowShortestPath/>
            <RepeatSequence name="unlimited_repeat">
                <Communicate/>
            </RepeatSequence>
        </ParallelAll>
     </BehaviorTree>
</root>
)";

//NOTE: IF YOU CHANGE CONVERGENCE THRESHOLD, IT MUST MATCH input.json
//NOTE: IF YOU CHANGE CONVERGENCE THRESHOLD, IT MUST MATCH input.json
//NOTE: IF YOU CHANGE CONVERGENCE THRESHOLD, IT MUST MATCH input.json
// Needed in input file as well because of access in cbba for at_consensus robot flag
// Can't easily pull that value into the xml, can do this later

double getCurrentTime() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() / 1000.0;
}


//void run_robot(int robot_id, std::string robot_type, Pose2D initial_pose, Pose2D goal_pose, cv::Scalar color, int step_size, Planner& planner, ShortestPath& shortest_path, CoveragePath& coverage_path, World& world, JSONParser& parser) {
void run_robot(int robot_id, std::string robot_type, Pose2D initial_pose, cv::Scalar color, int step_size, World& world) {
    std::cout << "Entering run_robot for robot " << robot_id << std::endl;

    try {
        std::cout << "Creating robot " << robot_id << " with step size " << step_size << "..." << std::endl;

        try {

            // Might be redundant to create another parser instance here, but doing so just in case threading causes issues
            std::string path = std::filesystem::current_path().append("src/simplified_marine_sim/config/input.json");
            JSONParser parser(path);

            auto robot_attributes = parser.j["robot_attributes"];
            const int obs_radius = robot_attributes["observation_radius"];

            // Need different instance for each robot so creating here
            Planner planner(step_size);
            ShortestPath shortest_path(step_size);
            CoveragePath coverage_path(step_size, obs_radius);

            Robot robot(&planner, &shortest_path, &coverage_path, &world, &parser, initial_pose, robot_id, robot_type, color);
            std::cout << "Robot " << robot_id << " created successfully." << std::endl;

            {
                world.plot();
                std::cout << "ID Check in Robot " << robot.getID() << std::endl;
                //robot.printTasksVector();
            }

            // Register Behavior Tree nodes
            std::cout << "Registering Behavior Tree nodes for robot " << robot_id << "..." << std::endl;
            BehaviorTreeFactory factory;

            try {
                //factory.registerNodeType<PlanShortestPath>("PlanShortestPath", std::ref(world), std::ref(robot), std::ref(shortest_path));
                //factory.registerNodeType<PlanCoveragePath>("PlanCoveragePath", std::ref(world), std::ref(robot), std::ref(coverage_path));
                //factory.registerNodeType<PlanRegroupPath>("PlanRegroupPath", std::ref(world), std::ref(robot), std::ref(shortest_path));
                //factory.registerNodeType<QueueSize<Pose2D>>("QueueSize");
                //factory.registerNodeType<RepeatNode>("RepeatNode");
                //factory.registerNodeType<PopFromQueue<Pose2D>>("PopFromQueue");
                //factory.registerNodeType<UseWaypoint>("UseWaypoint", std::ref(world), std::ref(robot));
                //factory.registerNodeType<SendMessage>("SendMessage", std::ref(world), std::ref(robot));
                //factory.registerNodeType<ReceiveMessage>("ReceiveMessage", std::ref(world), std::ref(robot));
                factory.registerNodeType<Communicate>("Communicate", std::ref(world), std::ref(robot));
                //factory.registerNodeType<TestMessages>("TestMessages", std::ref(world), std::ref(robot));
                factory.registerNodeType<NeedRegroup>("NeedRegroup", std::ref(robot));
                //factory.registerNodeType<TestCond>("TestCond", std::ref(robot));
                //factory.registerNodeType<RunTest>("RunTest");
                //factory.registerNodeType<RunTest2>("RunTest2");
                factory.registerNodeType<BuildBundle>("BuildBundle", std::ref(robot), std::ref(world), std::ref(parser));
                factory.registerNodeType<ResolveConflicts>("ResolveConflicts", std::ref(robot), std::ref(world), std::ref(parser));
                factory.registerNodeType<Ping>("Ping", std::ref(world), std::ref(robot));
                //factory.registerNodeType<DummySuccessAction>("DummySuccessAction");
                factory.registerNodeType<NewInfoAvailable>("NewInfoAvailable", std::ref(world), std::ref(robot));
                factory.registerNodeType<RepeatSequence>("RepeatSequence");
                factory.registerNodeType<CheckConvergence>("CheckConvergence", std::ref(world), std::ref(robot), std::ref(parser));
                factory.registerNodeType<GreedyTaskAllocator>("GreedyTaskAllocator", std::ref(robot), std::ref(world));
                factory.registerNodeType<FollowShortestPath>("FollowShortestPath", std::ref(robot), std::ref(world), std::ref(shortest_path));
                factory.registerNodeType<ExploreA>("ExploreA", std::ref(robot), std::ref(world));
                factory.registerNodeType<ExploreB>("ExploreB", std::ref(robot), std::ref(world));
                factory.registerNodeType<ExploreC>("ExploreC", std::ref(robot), std::ref(world));
                factory.registerNodeType<ExploreD>("ExploreD", std::ref(robot), std::ref(world));
                factory.registerNodeType<FollowCoveragePath>("FollowCoveragePath", std::ref(robot), std::ref(world), std::ref(coverage_path));
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
                BT::Blackboard::Ptr blackboard = BT::Blackboard::create(); // Testing
                std::cout << "====++++!!!!Created blackboard for robot " << robot_id << ": " << blackboard.get() << std::endl; 
                auto tree = factory.createTreeFromText(xml_text, blackboard); // Testing this too
                //auto tree = factory.createTreeFromText(xml_text); // Old way, results in some shared blackboard between robot threads
                std::cout << "Behavior tree created successfully for robot " << robot_id << "." << std::endl;

                // for more testing
                std::cout << "Robot " << robot_id << " tree instance: " << &tree << std::endl;
                auto root_node = tree.rootNode();
                std::cout << "Robot " << robot_id << " root node: " << root_node << std::endl;

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

        /*const int X = 400;
        const int Y = 400;
        const int step_size = 1;
        if (step_size != 1) {
            std::cout << "Your step size is larger than 1! Have you made changes in planners for smaller final step/neighbors etc.?" << std::endl;
            std::cin.get();
        }
        const double comms_range = 50.0; //310.0
        const int obs_radius = 4;*/

        std::string path = std::filesystem::current_path().append("src/simplified_marine_sim/config/input.json");
        JSONParser parser(path);

        // Get world attributes
        auto world_attributes = parser.j["world_attributes"];
        const int X = world_attributes["size_x"];
        const int Y = world_attributes["size_y"];
        const int step_size = world_attributes["step_size"];
        /*if (step_size != 1) {
            std::cout << "Your step size is larger than 1! Have you made changes in planners for smaller final step/neighbors etc.?" << std::endl;
            std::cin.get();
        }*/

        // Get robot attributes
        auto robot_attributes = parser.j["robot_attributes"];
        const double comms_range = robot_attributes["comms_range"];
        //const int obs_radius = robot_attributes["observation_radius"];

        //std::cout << "Right after parse, obs_radius: " << obs_radius << std::endl;

        Distance distance;
        SensorModel sensor_model(&distance);
        World world(X, Y, &distance, &sensor_model, &parser, comms_range);
        /*Planner planner(step_size);
        ShortestPath shortest_path(step_size);
        CoveragePath coverage_path(step_size, obs_radius);*/

        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::cout << "in main 1" << std::endl;
        std::unordered_map<std::string,std::vector<int>> test = world.getAllCapabilities();
        utils::printCapabilities(test);
        std::cout << "in main 2" << std::endl;
        std::cout << "Inits are done..." << std::endl;

        //auto agents = world.getAgents(); update to getAllAgentsInfo
        auto agents_info = world.getAllAgentsInfo();
        std::vector<std::thread> robot_threads;

        try {
            //for (const auto& agent : agents) { TODO update to traversed unordered map values (which are the agent structs now)
            for (const auto& pair : agents_info) {
                const auto& agent = pair.second;
                robot_threads.emplace_back(
                    run_robot, agent.id, agent.type, agent.initial_pose, agent.color,
                    step_size, std::ref(world)
                );
            }

            std::cout << "Threads started..." << std::endl;

            for (auto& thread : robot_threads) {
                thread.join();
            }
            
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