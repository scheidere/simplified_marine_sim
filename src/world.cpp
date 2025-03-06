#include <cstdio>
#include "distance.hpp"
#include "sensor_model.hpp"
#include "world.hpp"
#include "robot.hpp"
#include "parser.hpp"
#include "structs.hpp"
#include "utils.hpp"


struct Task;

World::World(int X, int Y, Distance* d, SensorModel* s, JSONParser* p, double comms_range) 
    : X(X),
    Y(Y), 
    distance(d), 
    sensor_model(s),
    parser(p),
    comms_range(comms_range),
    image(init())
{
    try {

        defineQuadrants();
        //initAllTasks();
        num_agents = parser->getNumAgents();
        all_agents_info = initAllAgentsInfo(); // pairs of agent id: struct
        num_tasks = parser->getNumLocalTasks();
        all_tasks_info = initAllTasksInfo();
        agent_indices = parser->getAgentIndices();
        agent_types = parser->getAgentTypes();
        task_types = parser->getTaskTypes();
        //std::cout << "start world init print" << std::endl;
        all_agent_capabilities = parser->getAgentCapabilities(agent_types, task_types);
        utils::printCapabilities(all_agent_capabilities);
        //std::cout << "end world init print" << std::endl;
        //print2DVector(agent_capabilities);
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in World constructor: " << e.what() << std::endl;
        throw; // Re-throw to propagate the exception
    }
}

cv::Mat& World::getImage() { 
    std::lock_guard<std::mutex> lock(world_mutex);
    return image; 
}

cv::Mat World::init() {
    try {
        std::cout << "Initializing world..." << std::endl;
        cv::Mat image(X, Y, CV_8UC3, cv::Scalar(255, 255, 255));
        return image;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in init: " << e.what() << std::endl;
        throw; // Re-throw to propagate the exception
    }
}


std::unordered_map<std::string,std::vector<int>> World::getAllCapabilities() {

    std::cout << "start world getter cap" << std::endl;
    utils::printCapabilities(all_agent_capabilities);
    std::cout << "end world getter cap" << std::endl;
    return all_agent_capabilities;
}

/*void initAllRobots(whattype run_robot) {

    if (parser.j.contains("agents") && parser.j["agents"].is_array()) {
        for (const auto& agent : j["agents"]) {
            agent_indices.push_back(agent["id"]);
            std::cout << agent["id"] << std::endl;
            std::thread robot1(run_robot, initial_pose1, goal_pose1, color1, step_size, std::ref(planner), std::ref(shortest_path), std::ref(coverage_path), std::ref(scorer), std::ref(world), std::ref(parser));

        }
    }
    else {
        std::cerr << "Error: agent info not found or invalid in JSON." << std::endl;
    }

}*/

// moved to structs.hpp
/*struct AgentInfo {
    int id;
    Pose2D initial_pose;
    cv::Scalar color;
};*/

//std::vector<AgentInfo> World::getAgents() {
std::unordered_map<int,AgentInfo> World::initAllAgentsInfo() {
    std::unordered_map<int,AgentInfo> all_agents_info;

    auto parsed_agents = parser->j["agents"]; // Assume parser extracts JSON info
    for (const auto& agent : parsed_agents) {
        int id = agent["id"].get<int>();
        AgentInfo agent_struct = {
            id,
            agent["type"],
            Pose2D{agent["start_x"].get<int>(), agent["start_y"].get<int>(), 0}, // Note we ignore the orientation theta for now
            Pose2D{agent["goal_x"].get<int>(), agent["goal_y"].get<int>(), 0}, // here too
            colors[id % colors.size()]  // Assign color cyclically
        };
        all_agents_info[id] = agent_struct;

    }
    return all_agents_info;
}

std::unordered_map<int,TaskInfo> World::initAllTasksInfo() {
    std::unordered_map<int,TaskInfo> all_tasks_info;

    auto parsed_tasks = parser->j["local_tasks"]; // Assume parser extracts JSON info
    for (const auto& task : parsed_tasks) {
        int id = task["id"].get<int>();
        TaskInfo task_struct = { // later will need to add checks to catch different types of task components that are or aren't present
            id,
            task["type"], // maybe can set up checks by type
            task["area"].get<std::unordered_map<std::string, int>>(),
            task["reward"]   
        };
        all_tasks_info[id] = task_struct;

    }
    return all_tasks_info;
}

std::vector<int> World::getRobotCapabilities(Robot* robot) {
    std::lock_guard<std::mutex> lock(world_mutex);

    //std::cout << "in world getRobotCapabilities " << std::endl;
    //std::cout << all_agent_capabilities.size() << std::endl;

    std::vector<int> doable_local_tasks;

    std::string agent_type = robot->getType();
    //std::cout << "Robot type: " << type << std::endl;
    //std::cin.get();

    if (all_agent_capabilities.find(agent_type) == all_agent_capabilities.end()) {
        std::cout << "Error: Capabilities for type " << agent_type << " not found!" << std::endl;
        return {};  // Return an empty vector
    }

    std::string log_msg = "Robot " + std::to_string(robot->getID()) + " is capable of the following tasks by id: ";
    //robot->log_info(log_msg);
    //for (const auto& elem : vec) {
    std::vector<int> robot_capabilities_by_type = all_agent_capabilities[agent_type]; 

    for (int i=0; i<task_types.size(); i++) {
        //robot->log("test2");
        //robot->log_info(std::to_string(i));
        //robot->log_info("Traversing all_task_info unordered_map...");
        for (auto& pair : all_tasks_info) {
            TaskInfo& local_task = pair.second;
            std::string log_msg3 = local_task.type;
            //robot->log_info(log_msg3);
            if (robot_capabilities_by_type[i]==1 && task_types[i]==local_task.type) {
                // Found doable task 
                std::string log_msg1 = "Found task of type " + local_task.type + " saving " + std::to_string(pair.first);  
                //robot->log_info(log_msg1);
                doable_local_tasks.push_back(pair.first); // pair.first is local task id (int)
                std::string log_msg2 = std::to_string(pair.first);
                //robot->log_info(log_msg2);
            }
        }


        //std::string log_msg2 = task_types[i] + ": " + std::to_string(robot_capabilities[i]);
        //robot->log(log_msg2);
    }

    // Since we traverse all_tasks_info which is an unordered map, tasks not necessarily traversed from id 1 and increasing
    // For simplicity, let's sort capabilities so id's are in increasing order
    std::sort(doable_local_tasks.begin(), doable_local_tasks.end());
    utils::log1DVector(doable_local_tasks, *robot);

    std::cout << "Doable local tasks vector: " << std::endl;
    utils::print1DVector(doable_local_tasks);

    return doable_local_tasks; // List of specific task indices of all doable types for given robot

}

std::unordered_map<int, Robot*>& World::getRobotTracker() { 
    std::lock_guard<std::mutex> lock(world_mutex);
    return robot_tracker;
}

std::unordered_map<int,std::vector<Msg>>& World::getMessageTracker() {
    std::lock_guard<std::mutex> lock(world_mutex);
    return message_tracker;
}

/*std::vector<Task>& World::getAllTasks() {
    std::lock_guard<std::mutex> lock(world_mutex);
    std::cout << "Returning all tasks. Number of tasks: " << allTasks.size() << std::endl;
    return allTasks;
}

int World::getTaskIndex(Task task_j) {
    std::lock_guard<std::mutex> lock(world_mutex);

    auto it = std::find_if(allTasks.begin(), allTasks.end(), [&](const Task& task) {
        return task.id == task_j.id;
    });

    int j = std::distance(allTasks.begin(), it);

    return j;
}*/

void World::clear(Pose2D pose) {
    std::lock_guard<std::mutex> lock(world_mutex);
    std::cout << "Clearing world..." << std::endl;
    cv::circle(image, cv::Point(pose.x, pose.y), 5, cv::Scalar(255, 255, 255), -1);
}

void World::plot() {
    std::lock_guard<std::mutex> lock(world_mutex);
    std::cout << "Plotting world..." << std::endl;

    // Plotting quadrant centers (for now)
    std::vector<Pose2D> quadrant_centers = getQuadrantCenters(); // also plots them; will remove once we add obstacles

    for (auto& pair : robot_tracker) {
        Robot* robot = pair.second;
        cv::Scalar color = robot->getColor();
        Pose2D robot_pose = robot->getPose();
        std::cout << "Plotting robot ID: " << robot->getID() << " at pose: " << robot_pose.x << ", " << robot_pose.y << " with color: " << color << std::endl;
        cv::circle(image, cv::Point(robot_pose.x, robot_pose.y), 5, color, -1);
    }

    cv::imshow("Quadrant Image", image);
    cv::waitKey(300);
}

void World::trackRobot(Robot* robot) {
    std::lock_guard<std::mutex> lock(world_mutex);
    std::cout << "Tracking Robot ID: " << robot->getID() << std::endl;
    robot_tracker[robot->getID()] = robot;
}

void World::printTrackedRobots() {
    std::lock_guard<std::mutex> lock(world_mutex);
    std::cout << "Tracked Robots:" << std::endl;
    for (const auto& ID_robo_pair : robot_tracker) {
        std::cout << " Raw Robot ID: " << ID_robo_pair.first << std::endl;
        if (ID_robo_pair.second != nullptr) {
            std::cout << " From Instance Robot ID: " << ID_robo_pair.second->getID() << std::endl;
            std::cout << " Pose: " << ID_robo_pair.second->getPose().x << ", " << ID_robo_pair.second->getPose().y << std::endl;
        } else {
            std::cout << " Null Robot instance" << std::endl;
        }
    }
}

void World::printMessageTracker() {
    std::lock_guard<std::mutex> lock(world_mutex);
    std::cout << "World message tracker:" << std::endl;
    for (auto& pair : message_tracker) {
        int receiverID = pair.first;
        std::vector<Msg>& messages = pair.second;

        std::cout << "Receiver ID: " << receiverID << std::endl;
        std::cout << "Number of messages for receiver: " << messages.size() << std::endl;
        for (auto& msg : messages) {
            //std::cout << "  From ID: " << msg.id << std::endl;
            std::cout << "Printing message..." << std::endl;
            printMessage(msg);
        }
    }
    std::cout << "End of printMessageTracker" << std::endl;
}

void World::printMessage(Msg msg) { // no mutex because used within the function above
    std::cout << "Message ID:" << msg.id << "\n";
    std::cout << "Task ID: " << msg.task_id << "\n";
    std::cout << "Location: (" << msg.location.x << ", " << msg.location.y << ", " << msg.location.theta << ")\n";
   /* std::cout << "Bundle: [";
    for (const auto& task : msg.bundle.tasks) {
        std::cout << task << " "; // Assuming tasks can be printed this way
    }
    std::cout << "]\n";*/
}

bool World::inComms(int id1, int id2) {
    std::lock_guard<std::mutex> lock(world_mutex);
    if (robot_tracker.find(id1) == robot_tracker.end() || robot_tracker.find(id2) == robot_tracker.end()) {
        return false;
    }
    Robot* robot1 = robot_tracker[id1];
    Robot* robot2 = robot_tracker[id2];
    Pose2D p1 = robot1->getPose(); Pose2D p2 = robot2->getPose();
    double distance_between_robots = distance->getEuclideanDistance(p1.x,p1.y,p2.x,p2.y);
    return distance_between_robots <= comms_range;
}

bool World::isCollision(int x, int y) {
    return {};
}

void World::defineQuadrants() {
    try {
        Pose2D tlA{0,0,0}; Pose2D blA{0,Y/2,0}; Pose2D trA{X/2,0,0}; Pose2D brA{X/2,Y/2,0};
        areaACoords.push_back(tlA); areaACoords.push_back(blA); areaACoords.push_back(trA); areaACoords.push_back(brA);
        Pose2D tlB{0,Y/2,0}; Pose2D blB{0,Y,0}; Pose2D trB{X/2,Y/2,0}; Pose2D brB{X/2,Y,0};
        areaBCoords.push_back(tlB); areaBCoords.push_back(blB); areaBCoords.push_back(trB); areaBCoords.push_back(brB);
        Pose2D tlC{X/2,0,0}; Pose2D blC{X/2,Y/2,0}; Pose2D trC{X,0,0}; Pose2D brC{X,Y/2,0};
        areaCCoords.push_back(tlC); areaCCoords.push_back(blC); areaCCoords.push_back(trC); areaCCoords.push_back(brC);
        Pose2D tlD{X/2,Y/2,0}; Pose2D blD{X/2,Y,0}; Pose2D trD{X,Y/2,0}; Pose2D brD{X,Y,0};
        areaDCoords.push_back(tlD); areaDCoords.push_back(blD); areaDCoords.push_back(trD); areaDCoords.push_back(brD);

        std::cout << "Quadrants defined." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in defineQuadrants: " << e.what() << std::endl;
        throw; // Re-throw to propagate the exception
    }
}

std::vector<Pose2D> World::getQuadrantCenters() {
    std::vector<Pose2D> quadrant_centers;
    try {
        Pose2D centerA{X/4,X/4,0};
        Pose2D centerB{X/4,3*X/4,0};
        Pose2D centerC{3*X/4,X/4,0};
        Pose2D centerD{3*X/4,3*X/4,0};
        quadrant_centers.push_back(centerA);
        quadrant_centers.push_back(centerB);
        quadrant_centers.push_back(centerC);
        quadrant_centers.push_back(centerD);

        int size = 5;

        cv::rectangle(image, cv::Point(centerA.x - size, centerA.y - size), cv::Point(centerA.x + size, centerA.y + size), cv::Scalar(0, 0, 0), -1);
        cv::rectangle(image, cv::Point(centerB.x - size, centerB.y - size), cv::Point(centerB.x + size, centerB.y + size), cv::Scalar(0, 0, 0), -1);
        cv::rectangle(image, cv::Point(centerC.x - size, centerC.y - size), cv::Point(centerC.x + size, centerC.y + size), cv::Scalar(0, 0, 0), -1);
        cv::rectangle(image, cv::Point(centerD.x - size, centerD.y - size), cv::Point(centerD.x + size, centerD.y + size), cv::Scalar(0, 0, 0), -1);

        std::cout << "Quadrant centers defined: " << quadrant_centers.size() << " centers." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Exception caught in getQuadrantCenters: " << e.what() << std::endl;
        throw; // Re-throw to propagate the exception
    }
    return quadrant_centers;
}

/*void World::initAllTasks() {
    try {
        std::vector<Pose2D> quadrant_centers = getQuadrantCenters();

        if (quadrant_centers.size() < 4) {
            throw std::runtime_error("Not enough quadrant centers to initialize tasks.");
        }

        Task exploreA(1, "Explore area A", quadrant_centers[0], 0, 0, 0); 
        allTasks.push_back(exploreA);
        Task exploreB(2, "Explore area B", quadrant_centers[1], 0, 0, 0); 
        allTasks.push_back(exploreB);
        Task exploreC(3, "Explore area C", quadrant_centers[2], 0, 0, 0); 
        allTasks.push_back(exploreC);
        Task exploreD(4, "Explore area D", quadrant_centers[3], 0, 0, 0); 
        allTasks.push_back(exploreD);

        std::cout << "Initialized all tasks. Number of tasks: " << allTasks.size() << std::endl;

        // Check the size of allTasks
        if (allTasks.size() > allTasks.max_size()) {
            throw std::length_error("The number of tasks exceeds the maximum allowable size.");
        }
    } catch (const std::length_error& e) {
        std::cerr << "std::length_error caught in initAllTasks: " << e.what() << std::endl;
        throw; // Re-throw to propagate the exception
    } catch (const std::runtime_error& e) {
        std::cerr << "std::runtime_error caught in initAllTasks: " << e.what() << std::endl;
        throw; // Re-throw to propagate the exception
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in initAllTasks: " << e.what() << std::endl;
        throw; // Re-throw to propagate the exception
    }
}*/