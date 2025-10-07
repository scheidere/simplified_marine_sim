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

        // Get filename for logging and then open it
        std::string filename = generateLogFilename();
        std::ofstream clear(filename, std::ios::out); // Clear logging file from previous run
        clear.close();
        world_log.open(filename, std::ios::app); // Allow appending

        start_time = std::chrono::steady_clock::now();

        defineQuadrants();
        //initAllTasks();
        num_agents = parser->getNumAgents();
        all_agents_info = initAllAgentsInfo(); // pairs of agent id: struct
        num_tasks = parser->getNumLocalTasks();
        all_tasks_info = initAllTasksInfo();
        //agent_indices = parser->getAgentIndices();
        agent_types = parser->getAgentTypes();
        task_types = parser->getTaskTypes();
        //std::cout << "start world init print" << std::endl;
        all_agent_capabilities = parser->getAgentCapabilities(agent_types, task_types);
        initMessageTracker();
        //initPingIDTracker();
        initPingTracker(); // Note: Pings contain sender id and time sender last updated its own beliefs via CBBA
        //std::cout << "end world init print" << std::endl;
        //print2DVector(agent_capabilities);

        // Adding this so that greedy alg timing is consistent, because unlike CBBA/CBGA BTs, there isn't sufficient time to load world fully before running alg
        // Want to get consistent estimate of runtime for greedy
        /*for (const auto& task : all_tasks_info) {
            getTaskInfo(task.first);  // Force initialization of all tasks
        }
        for (const auto& task : all_tasks_info) {
            int task_id = task.first;
            // Call the exact same methods that getTaskScore() calls
            getTaskReward(task_id);
            getTaskLocation(task_id);
        }*/

    } catch (const std::exception& e) {
        std::cerr << "Exception caught in World constructor: " << e.what() << std::endl;
        throw; // Re-throw to propagate the exception
    }
}

std::chrono::steady_clock::time_point World::getStartTime() const {
    return start_time;
}

std::string World::generateLogFilename() {
    std::ostringstream oss;
    oss << "world_log.txt";
    return oss.str();
}

void World::log_info(std::string log_msg) {
    std::lock_guard<std::mutex> lock(world_mutex);

    if (world_log.is_open()) {

        world_log << log_msg << std::endl;

    } else {
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Failed to open log file for world..." << std::endl;
    }

}

void World::logCurrentTeamAssignment() {

    log_info("Paths for each agent on the team:");

    for (auto& pair : robot_tracker) {
        int id = pair.first;
        Robot* robot = pair.second;
        std::vector<int>& path = robot->getPath();
        std::string bla = std::string("Robot ") + std::to_string(id) + ": ";
        log_info(bla);
        utils::log1DVectorFromWorld(path, *this);

    }
}

void World::logCurrentTeamTaskProgress() {

    log_info("Current task progress tracker for each agent on the team:");
    for (auto& pair : robot_tracker) {
        int id = pair.first;
        Robot* robot = pair.second;
        std::string bla = std::string("Robot ") + std::to_string(id) + ": ";
        log_info(bla);
        utils::logUnorderedMapWorld(robot->getTaskProgress(), *this);

        std::cout.flush();
        std::cout << std::flush;
        std::cerr << std::flush;
    }

    log_info("Finished logging task progress");
    std::cout << std::flush;

}

double World::getElapsedTime() const {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now - start_time).count();
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

void World::initMessageTracker() {
    for (const auto& [robot_id, _] : all_agents_info) {
        message_tracker[robot_id] = {};  // Initialize with an empty vector
    }
}

void World::initPingTracker() {
    for (const auto& [robot_id, _] : all_agents_info) {
        ping_tracker[robot_id] = {};  // Initialize with an empty vector
    }
}

std::unordered_map<std::string,std::vector<int>> World::getAllCapabilities() {

    //std::cout << "start world getter cap" << std::endl;
    //utils::printCapabilities(all_agent_capabilities);
    //std::cout << "end world getter cap" << std::endl;
    return all_agent_capabilities;
}

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
            //Pose2D{agent["goal_x"].get<int>(), agent["goal_y"].get<int>(), 0}, // here too
            colors[id % colors.size()]  // Assign color cyclically
        };
        all_agents_info[id] = agent_struct;

    }
    return all_agents_info;
}

int World::getGroupSize(std::unordered_map<std::string, int> group_info) {

    int group_size = 0;

    for (const auto& robot_type_num_pair : group_info) { // for each robot type, count how many required for group and sum
        group_size += robot_type_num_pair.second;  // add num of robots for current robot type
    }

    return group_size;
}

std::unordered_map<int, TaskInfo> World::initAllTasksInfo() {
    std::unordered_map<int, TaskInfo> all_tasks_info;

    auto parsed_tasks = parser->j["local_tasks"]; // Assume parser extracts JSON info
    for (const auto& task : parsed_tasks) {
        int id = task["id"].get<int>();
        std::string name = task["name"];
        std::string type = task["type"];
        std::unordered_map<std::string, int> group_info = task["group_info"].get<std::unordered_map<std::string, int>>();
        int group_size = getGroupSize(group_info); // sum of all the sub groups in group_info, by agent type
        double reward = task["reward"];

        std::unordered_map<std::string, int> area; // This will remain empty if no area defined in input json
        std::pair<int, int> location = {-1, -1};

        if (task.contains("area")) {
            area = task["area"].get<std::unordered_map<std::string, int>>();
        }

        if (task.contains("location")) {
            auto loc_json = task["location"];
            int x = loc_json.value("x", -1);  // fallback to -1 if missing
            int y = loc_json.value("y", -1);
            location = std::make_pair(x, y);
        }

        TaskInfo task_struct = {
            id, // int
            name, // string
            type, // string
            group_size, // int
            group_info, // <std::unordered_map<std::string, int>>
            //prerequisute_task_failures, // will figure out later
            location, // std::pair<int,int>
            area, // <std::unordered_map<std::string, int>>
            reward // double
        };

        all_tasks_info[id] = task_struct;
    }

    return all_tasks_info;
}

TaskInfo& World::getTaskInfo(int task_id) {
    std::lock_guard<std::mutex> lock(world_mutex);

    return all_tasks_info.at(task_id);

    /*if (all_tasks_info.find(task_id) != all_tasks_info.end()) {
        return all_tasks_info.at(task_id);
    } else {
        throw std::runtime_error("Task ID not found in all_tasks_info");
    }*/
}

// TaskInfo& World::getTaskInfoUnsafe(int task_id) {

//     return all_tasks_info.at(task_id);

//     /*if (all_tasks_info.find(task_id) != all_tasks_info.end()) {
//         return all_tasks_info.at(task_id);
//     } else {
//         throw std::runtime_error("Task ID not found in all_tasks_info");
//     }*/
// }

AgentInfo& World::getAgentInfo(int agent_id) {
    std::lock_guard<std::mutex> lock(world_mutex);

    return all_agents_info.at(agent_id); // Simpler way

    /*if (all_agents_info.find(agent_id) != all_agents_info.end()) {
        return all_agents_info.at(agent_id);
    } else {
        throw std::runtime_error("Agent ID not found in all_agents_info");
    }*/
}

std::string& World::getAgentType(int agent_id) {
    // No mutex to avoid deadlock because getAgentInfo covers it
    
    AgentInfo& agent_info = getAgentInfo(agent_id);
    return agent_info.type;
}

int& World::getTaskGroupSize(int task_id) {
    // No mutex to avoid deadlock because getTaskInfo covers it
    
    TaskInfo& task_info = getTaskInfo(task_id);
    return task_info.group_size;
}

std::unordered_map<std::string, int>& World::getTaskGroupInfo(int task_id) {
    // No mutex to avoid deadlock because getTaskInfo covers it
    
    TaskInfo& task_info = getTaskInfo(task_id);
    return task_info.group_info;
}

double& World::getTaskReward(int task_id) {
    // No mutex to avoid deadlock because getTaskInfo covers it
    
    TaskInfo& task_info = getTaskInfo(task_id);
    return task_info.reward;
}

// double& World::getTaskRewardUnsafe(int task_id) {
//     // No mutex to avoid deadlock because getTaskInfo covers it
    
//     TaskInfo& task_info = getTaskInfoUnsafe(task_id);
//     return task_info.reward;
// }

//std::pair<int, int> World::getTaskLocation(int task_id, Robot* robot) { // Robot passed in for TESTING ONLY
std::pair<int, int> World::getTaskLocation(int task_id) {
    // No mutex to avoid deadlock because getTaskInfo covers it

    TaskInfo& task_info = getTaskInfo(task_id);

    //robot->log_info("in getTaskLocation"); // TESTING ONLY, needs robot as input

    std::string bla = "x: " + std::to_string(task_info.location.first) + ", y: " + std::to_string(task_info.location.second);
    //robot->log_info(bla); // TESTING ONLY, needs robot as input

    // If task location is explicitly set
    if (task_info.location.first != -1 && task_info.location.second != -1) {

        //robot->log_info("task location explicitly exists (not quadrant-defined) and found"); // TESTING ONLY, needs robot as input

        int x = task_info.location.first;
        int y = task_info.location.second;
        return std::make_pair(x, y);
    } 
    // If location is defined via quadrant area
    else if (!task_info.area.empty()) {
        //robot->log_info("task location must be calculated from quadrant given as area"); // TESTING ONLY, needs robot as input
        return getTaskLocationFromArea(task_info.area); // pass the area map
    } 
    // If no location information exists
    else {
        throw std::runtime_error("Task has no location or area defined.");
    }
}

// std::pair<int, int> World::getTaskLocationUnsafe(int task_id) {
//     // No mutex to avoid deadlock because getTaskInfo covers it

//     TaskInfo& task_info = getTaskInfoUnsafe(task_id);

//     //robot->log_info("in getTaskLocation"); // TESTING ONLY, needs robot as input

//     std::string bla = "x: " + std::to_string(task_info.location.first) + ", y: " + std::to_string(task_info.location.second);
//     //robot->log_info(bla); // TESTING ONLY, needs robot as input

//     // If task location is explicitly set
//     if (task_info.location.first != -1 && task_info.location.second != -1) {

//         //robot->log_info("task location explicitly exists (not quadrant-defined) and found"); // TESTING ONLY, needs robot as input

//         int x = task_info.location.first;
//         int y = task_info.location.second;
//         return std::make_pair(x, y);
//     } 
//     // If location is defined via quadrant area
//     else if (!task_info.area.empty()) {
//         //robot->log_info("task location must be calculated from quadrant given as area"); // TESTING ONLY, needs robot as input
//         return getTaskLocationFromArea(task_info.area); // pass the area map
//     } 
//     // If no location information exists
//     else {
//         throw std::runtime_error("Task has no location or area defined.");
//     }
// }

std::pair<int,int> World::getTaskLocationFromArea(std::unordered_map<std::string, int>& area) {
    // No mutex to avoid deadlock because getTaskInfo covers it

    int xmin = area.at("xmin");; int xmax = area.at("xmax");; int ymin = area.at("ymin");; int ymax = area.at("ymax");
    int xcenter = (xmin + xmax) / 2; int ycenter = (ymin + ymax) / 2; 
    std::pair<int,int> center = std::make_pair(xcenter,ycenter);

    return center;
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
        std::cerr << "Error: Capabilities for type " << agent_type << " not found!" << std::endl;
        return {};  // Return an empty vector
    }

    // utils::log1DVector(task_types, *robot);

    std::string log_msg = "Robot " + std::to_string(robot->getID()) + " is capable of the following tasks by id: ";
    //robot->log_info(log_msg);
    //for (const auto& elem : vec) {
    robot->log_info("all_agent_capabilities: ");
    utils::logMapOfVectors(all_agent_capabilities, *robot);
    std::vector<int> robot_capabilities_by_type = all_agent_capabilities[agent_type]; 

    robot->log_info("robot_capabilities_by_type: ");
    utils::log1DVector(robot_capabilities_by_type, *robot);

    for (int i=0; i<task_types.size(); i++) {
        //robot->log("test2");
        //robot->log_info(std::to_string(i));
        //robot->log_info("Traversing all_task_info unordered_map...");
        for (auto& pair : all_tasks_info) {
            TaskInfo& local_task = pair.second;
            std::string log_msg3 = local_task.type;
            //robot->log_info(log_msg3);
            
            // Check if robot can do specific task (solo or co-op) and task type must match by definition
            if ((robot_capabilities_by_type[i]==1 || robot_capabilities_by_type[i] == 2) && task_types[i]==local_task.type) {
                // Found doable task 
                std::string log_msg1 = "Found task of type " + local_task.type + " saving " + std::to_string(pair.first);  
                robot->log_info(log_msg1);
                doable_local_tasks.push_back(pair.first); // pair.first is local task id (int)
                std::string log_msg2 = std::to_string(pair.first);
                robot->log_info(log_msg2);
            }
        }


        //std::string log_msg2 = task_types[i] + ": " + std::to_string(robot_capabilities[i]);
        //robot->log(log_msg2);
    }

    // Since we traverse all_tasks_info which is an unordered map, tasks not necessarily traversed from id 1 and increasing
    // For simplicity, let's sort capabilities so id's are in increasing order
    std::sort(doable_local_tasks.begin(), doable_local_tasks.end());
    utils::log1DVector(doable_local_tasks, *robot);

    //std::cout << "Doable local tasks vector: " << std::endl;
    utils::print1DVector(doable_local_tasks);

    return doable_local_tasks; // List of specific task indices of all doable types for given robot

}

// std::unordered_map<int, Robot*>& World::getRobotTracker() { 
//     std::lock_guard<std::mutex> lock(world_mutex);
//     return robot_tracker;
// }

// updated to map for deterministic order in message broadcasting
std::map<int, Robot*>& World::getRobotTracker() { 
    std::lock_guard<std::mutex> lock(world_mutex);
    return robot_tracker;
}

std::unordered_map<int,std::vector<Msg>>& World::getMessageTracker() {
    std::lock_guard<std::mutex> lock(world_mutex);
    return message_tracker;
}

std::unordered_map<int,std::vector<Msg>>& World::getMessageTrackerUnsafe() {
    return message_tracker;  // No mutex
}

std::unordered_map<int,std::vector<std::pair<int,double>>>& World::getPingTracker() {
    std::lock_guard<std::mutex> lock(world_mutex);
    return ping_tracker;
}

void World::clear(Pose2D pose) {
    std::lock_guard<std::mutex> lock(world_mutex);
    std::cout << "Clearing world..." << std::endl;
    cv::circle(image, cv::Point(pose.x, pose.y), 5, cv::Scalar(255, 255, 255), -1);
}

/*void World::plot() {
    std::lock_guard<std::mutex> lock(world_mutex);
    //std::cout << "Plotting world..." << std::endl;

    // Plotting quadrant centers (for now)
    std::vector<Pose2D> quadrant_centers = getQuadrantCenters(); // also plots them; will remove once we add obstacles

    for (auto& pair : robot_tracker) {
        Robot* robot = pair.second;
        cv::Scalar color = robot->getColor();
        Pose2D robot_pose = robot->getPose();
        //std::cout << "Plotting robot ID: " << robot->getID() << " at pose: " << robot_pose.x << ", " << robot_pose.y << " with color: " << color << std::endl;
        cv::circle(image, cv::Point(robot_pose.x, robot_pose.y), 5, color, -1);
    }

    cv::imshow("Quadrant Image", image);
    cv::waitKey(300);
}*/

void World::plot() {
    // Mutex used to lock whole function but there were big delays for some of the robots

    std::vector<std::pair<Pose2D, cv::Scalar>> robot_data;
    
    {  // Mutex locking getPose, getColor calls but NOT the time-consuming drawing part (circles get left as strays if don't protect at all)
        std::lock_guard<std::mutex> lock(world_mutex);
        
        // Collect all data while holding lock
        std::vector<Pose2D> quadrant_centers = getQuadrantCenters();
        
        for (auto& pair : robot_tracker) {
            Robot* robot = pair.second;
            robot_data.push_back({robot->getPose(), robot->getColor()});
        }
    }  // Lock released here
    
    // Do all the slow drawing/display work without holding lock
    for (auto& data : robot_data) {
        cv::circle(image, cv::Point(data.first.x, data.first.y), 5, data.second, -1);
    }
    
    cv::imshow("Quadrant Image", image);
    cv::waitKey(300);  // No longer blocking other robots!
}

void World::trackRobot(Robot* robot) {
    std::lock_guard<std::mutex> lock(world_mutex);
    //std::cout << "Tracking Robot ID: " << robot->getID() << std::endl;
    robot_tracker[robot->getID()] = robot;
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
    return distance_between_robots <= comms_range; // before testing below

    // below for testing
    /*bool result = distance_between_robots <= comms_range;
    
    std::cout << "inComms(" << id1 << ", " << id2 << "): distance=" 
              << distance_between_robots << ", result=" << result << std::endl;
    
    return result;*/
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

        //std::cout << "Quadrants defined." << std::endl;
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

        //std::cout << "Quadrant centers defined: " << quadrant_centers.size() << " centers." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Exception caught in getQuadrantCenters: " << e.what() << std::endl;
        throw; // Re-throw to propagate the exception
    }
    return quadrant_centers;
}

std::vector<int> World::getNeighborsInComms(int robot_id) {

    // Tested

    // Return vector of robot IDs, one for each robot in comms with given robot ID

    std::vector<int> neighbor_ids;

    for (auto& pair : robot_tracker) {
        int other_robot_id = pair.first;
        Robot* robot = pair.second;
        if ( other_robot_id != robot_id && inComms(robot_id,other_robot_id)) {
            // Neighbor found! Potential neighbor is not self and is in comms range of robot with id = robot_id so it is a neighbor
            neighbor_ids.push_back(other_robot_id);
        }
    }

    return neighbor_ids;

    //return ping_id_tracker[robot_id]; // old way, but want to avoid pinging for now
}

double World::getMaxNeighborTimestamp(int id_i, int id_k) {

    // Tested

    // Given k, the id of a robot not within comms with current robot with id i
    // Return the maximum (latest) timestamp that a neighbor of (in comms with) i received a message from k
    // If none of the neighbor has no msg from k, return -1

    double max_timestamp = -1.0;

    std::vector<int> neighbors_of_i = getNeighborsInComms(id_i);

    Robot* robot = robot_tracker.at(id_i); // just for testing
    robot->log_info("In getMaxNeighborTimestamp() in world...");
    std::string bla = "Neighbors of robot " + std::to_string(id_i);
    robot->log_info(bla);
    utils::log1DVector(neighbors_of_i, *robot);

    robot->log_info("Timestamps BEFORE change in world::getMaxNeighborTimestamp:");
    utils::logUnorderedMap(robot->getTimestamps(),*robot);

    std::string blork1 = "id_i: " + std::to_string(id_i);
    robot->log_info(blork1);
    std::string blork = "id_k: " + std::to_string(id_k);
    robot->log_info(blork);

    for (int id_m : neighbors_of_i) {
        for (Msg msg : message_tracker[id_m]) {
            if (msg.id == id_k) { // Found robot m in comms with both i and k, that has received a msg from k
                // Found message from k
                if (msg.timestamps[id_k] > max_timestamp) { // Found more recent timestamp of info received by m from k
                    max_timestamp = msg.timestamps[id_k];
                }
            }

        }
    }

    robot->log_info("Timestamps AFTER change in world::getMaxNeighborTimestamp:");
    utils::logUnorderedMap(robot->getTimestamps(),*robot);

    return max_timestamp;
}

bool World::hasTaskInfo(int task_id) {
    return all_tasks_info.find(task_id) != all_tasks_info.end();
}

// for debugging greedy inconsistent runtime (was due to world not being "pre-warmed")
void World::debugTaskAccess(int task_id, Robot& robot) {
    auto start = std::chrono::high_resolution_clock::now();
    
    robot.log_info("Getting task info for task " + std::to_string(task_id) + "...");
    TaskInfo& info = getTaskInfo(task_id);
    
    auto mid = std::chrono::high_resolution_clock::now();
    
    robot.log_info("Getting location for task " + std::to_string(task_id) + "...");
    if (info.location.first == -1) {
        robot.log_info("Location not set, calculating from area...");
        auto loc = getTaskLocationFromArea(info.area);
        robot.log_info("Calculated location: (" + std::to_string(loc.first) + ", " + std::to_string(loc.second) + ")");
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    
    double info_time = std::chrono::duration<double>(mid - start).count();
    double loc_time = std::chrono::duration<double>(end - mid).count();
    
    robot.log_info("Task " + std::to_string(task_id) + " - info: " + std::to_string(info_time) + "s, location: " + std::to_string(loc_time) + "s");
}

bool World::clearPathFullGroupPresent(int current_task_id) {

    // Get task struct
    TaskInfo& current_task = getTaskInfo(current_task_id); // Get task struct from world 
    std::pair<int,int> task_location = current_task.location;

    // Since the type requirements for robots in the group is handled in resolveConflicts, robot.trackAssignedRobotsbySubGroup(...) and CBGA.isGroupEffectivelyFull(...)
    // Here we only check that total number in group required is present (we assume they are the correct types since determined prior to task allocation)

    int count_robots_at_location = 0; // Consider robots doing clear path task that are at location of clearing
    for (auto& pair : robot_tracker) {
        int robot_id = pair.first;
        Robot* robot = pair.second;

        Pose2D current_pose = robot->getPose();
        std::pair<int,int> current_loc = {current_pose.x, current_pose.y};

        if (current_loc == task_location) {
            count_robots_at_location += 1;
        }
    }

    if (count_robots_at_location == current_task.group_size) {
        // Full group present to start task
        return true;
    }

    // Full group not yet present
    return false;
}

void World::updateTaskCompletionLog(int robot_id, int completed_task_id) {

    task_completion_log[robot_id].push_back(completed_task_id);

    task_completion_order.push_back(std::make_pair(robot_id, completed_task_id));
}

void World::logTaskCompletion() {

    utils::logMapOfVectorsWorld(task_completion_log, *this);

    // Below for actual order, robot id does task id
    for (const auto& pair : task_completion_order) {
        std::string yeup = "Robot " + std::to_string(pair.first) + " did task " + std::to_string(pair.second);
        log_info(yeup);
    }
}

void World::updateCumulativeReward(double reward) {
    std::lock_guard<std::mutex> lock(world_mutex);

    cumulative_reward_achieved += reward;
}

void World::updateCumulativeDistance() {

    // Each robot cumulatively tracks total distance traveled (in robot.move())
    // Here we just add those distances for all robots on the team to get the current distance traveled by team thus far
    // Only robot 1 logs this so no mutex here

    log_info("in world.getCumulativeDistance");

    double new_cumulative_distance;
    for (auto& pair : robot_tracker) {
        int id = pair.first;
        Robot* robot = pair.second;
        new_cumulative_distance += robot->getCumulativeDistance();
    }

    // Update team's cumulative distance stored in world
    cumulative_distance_traveled = new_cumulative_distance;
    std::string d = "Cumulative distance in world func: " + std::to_string(cumulative_distance_traveled);
    log_info(d);
}