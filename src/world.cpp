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
    image(init()),
    background_image(image.clone()) // testing this
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
        num_tasks = parser->getNumLocalTasks() + parser->getNumSubtasks(); // Adding subtasks because they need to be considered in winning bids matrix for failure handling
        all_tasks_info = initAllTasksInfo();
        all_subtasks_info = initAllSubtasksInfo();


        agent_ids = getAgentIDs();
        subtask_ids = getSubtaskIDs();

        signal_path_loss_factor = initSignalPathLossFactor();
        // std::string y = "signal_path_loss_factor:" + std::to_string(signal_path_loss_factor);
        // log_info(y);
        decay_rate = initDecayRate();
        fault_percentage = initFaultInjectionPercentage();
        random_seed = initRandomSeed();

        fault_injection_tracker = initManualFaultInjectionTracker(); // only if fault_injection given manually in input world attributes
        //fault_injection_tracker = initFaultInjectionTracker(); // Init'd randomly by input fault percentage

        log_info("all_tasks_info: ");
        logListofTaskIDs(all_tasks_info);
        log_info("all_subtasks_info: ");
        logListofTaskIDs(all_subtasks_info);
        log_info("fault_injection_tracker: ");
        utils::logUnorderedMapWorld(fault_injection_tracker, *this);

        //addObstacle({{50, 50}, {70, 50}, {60, 70}}); // for testing
        getObstacles(); // parse and populate obstacles list
        getUnknownObstacles(); // get obstacles that are not known by robot until encountered
        initializeBackground(); // just obstacles, so don't have to replot

        log_info("after obstacle init");

        //agent_indices = parser->getAgentIndices();
        agent_types = parser->getAgentTypes();
        task_types = parser->getTaskTypes();
        //std::cout << "start world init print" << std::endl;
        all_agent_capabilities = parser->getAgentCapabilities(agent_types, task_types);
        log_info("all_agents_capabilitiesss:");
        utils::log2DUnorderedMap(all_agent_capabilities, *this);
        initMessageTracker();
        //initPingIDTracker();
        initPingTracker(); // Note: Pings contain sender id and time sender last updated its own beliefs via CBBA
        //std::cout << "end world init print" << std::endl;
        //print2DVector(agent_capabilities);
        log_info("end of world constructor");

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

        home_pose = initHomePose();


    } catch (const std::exception& e) {
        std::cerr << "Exception caught in World constructor: " << e.what() << std::endl;
        throw; // Re-throw to propagate the exception
    }
}

Pose2D World::initHomePose() {
    auto world_attrs = parser->j["world_attributes"];
    if (world_attrs.contains("home_loc")) {
        auto home_loc = world_attrs["home_loc"];
        return {home_loc["x"].get<int>(), 
                home_loc["y"].get<int>(), 
                0};
    }
    return {0, 0, 0};
}

std::vector<int> World::getAgentIDs() {

    std::vector<int> agent_ids;

    for (const auto& [robot_id, _] : all_agents_info) {
        agent_ids.push_back(robot_id);
    }

    std::sort(agent_ids.begin(), agent_ids.end());
    return agent_ids;
}

std::vector<int> World::getSubtaskIDs() {

    std::vector<int> subtask_ids;

    for (const auto& [subtask_id, _] : all_subtasks_info) {
        subtask_ids.push_back(subtask_id);
    }

    std::sort(subtask_ids.begin(), subtask_ids.end());

    return subtask_ids;
}

void World::logListofTaskIDs(std::unordered_map<int,TaskInfo> task_list) {
    std::string log_msg = "Task IDs: ";
    for (const auto& [id, task] : task_list) {
        log_msg += std::to_string(id) + " ";
    }
    log_info(log_msg);
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

    log_info("Bundles for each agent on the team:");

    for (auto& pair : robot_tracker) {
        int id = pair.first;
        Robot* robot = pair.second;
        std::vector<int>& bundle = robot->getBundle();
        std::string bla = std::string("Robot ") + std::to_string(id) + ": ";
        log_info(bla);
        utils::log1DVectorFromWorld(bundle, *this);

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

// std::unordered_map<std::string,std::vector<int>> World::getAllCapabilities() {
std::unordered_map<std::string, std::unordered_map<std::string,bool>> World::getAllCapabilities() {

    //std::cout << "start world getter cap" << std::endl;
    //utils::printCapabilities(all_agent_capabilities);
    //std::cout << "end world getter cap" << std::endl;
    return all_agent_capabilities;
}

//std::vector<AgentInfo> World::getAgents() {
// random colors
/*std::unordered_map<int,AgentInfo> World::initAllAgentsInfo() {
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
}*/

std::unordered_map<int,AgentInfo> World::initAllAgentsInfo() {
    std::unordered_map<int,AgentInfo> all_agents_info;
    auto parsed_agents = parser->j["agents"]; // Assume parser extracts JSON info
    for (const auto& agent : parsed_agents) {
        int id = agent["id"].get<int>();
        
        // Get color from JSON if it exists, otherwise use default cyclic color
        cv::Scalar agent_color;
        if (agent.contains("color") && agent["color"].is_array() && agent["color"].size() == 3) {
            agent_color = cv::Scalar(
                agent["color"][0].get<int>(),
                agent["color"][1].get<int>(),
                agent["color"][2].get<int>()
            );
        } else {
            agent_color = colors[id % colors.size()];  // Fallback to cyclic assignment
        }
        
        AgentInfo agent_struct = {
            id,
            agent["type"],
            Pose2D{agent["start_x"].get<int>(), agent["start_y"].get<int>(), 0}, // Note we ignore the orientation theta for now
            agent_color
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
    auto parsed_tasks = parser->j["tasks"];
    
    for (const auto& task : parsed_tasks) {
        
        int id = task["id"].get<int>();
        std::string name = task["name"];
        std::string type = task["type"];
        std::vector<int> subtasks = {}; // Changed from string to int for access ease
        if (task.contains("actions")) { // previously "subtasks"
            subtasks = task["actions"].get<std::vector<int>>();
        }
        int main_id = -1; // This is only for subtasks, would be redundant with own id for main tasks
        
        std::unordered_map<std::string, int> group_info = task["group_info"].get<std::unordered_map<std::string, int>>();
        
        int group_size = getGroupSize(group_info);
        
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

        // Info that only pertains to actions, not main tasks being initialized here
        int prerequisite_failures = -1; // is this needed?
        // we don't give a dummy value for main_id which is only for actions not tasks sooo

        TaskInfo task_struct = {
            id, // int
            name, // string
            type, // string
            prerequisite_failures, // int
            subtasks, // std::vector<int>, this is the actions list, calling subtasks still because it's many changes to update throughout
            main_id, // int
            group_size, //int
            group_info, // <std::unordered_map<std::string, int>>
            location, // std::pair<int,int>
            area, // <std::unordered_map<std::string, int>>
            reward // double
        };

        all_tasks_info[id] = task_struct;
    }

    log_info("all_tasks_info:");
    utils::logAllTasksInfo(all_tasks_info, *this);

    return all_tasks_info;

}

std::unordered_map<int, TaskInfo> World::initAllSubtasksInfo() {
    std::unordered_map<int, TaskInfo> all_subtasks_info;
    auto parsed_actions = parser->j["actions"];
    auto parsed_tasks = parser->j["tasks"];

    //std::cout << "Starting subtasks initialization..." << std::endl;
    
    for (const auto& action : parsed_actions) {
        
        // Info from action definition only
        int id = action["id"].get<int>();
        std::string name = action["name"];
        int prerequisite_failures = action["prerequisite_failures"];
        int main_id = action["main_id"].get<int>();
        
        // Info from parent/main task that given action is a subtask of
        std::string type = "";
        std::vector<int> subtasks = {};
        std::unordered_map<std::string, int> group_info;
        int group_size = 0;
        std::unordered_map<std::string, int> area;
        std::pair<int, int> location = {-1, -1};
        double reward = 0.0;

        if (action.contains("location")) {
                    auto loc_json = action["location"];
                    int x = loc_json.value("x", -1);  // fallback to -1 if missing
                    int y = loc_json.value("y", -1);
                    location = std::make_pair(x, y);
        }

        for (const auto& task : parsed_tasks) {
            if (task["id"] == main_id) { // found relevant parent/main task to given action
                log_info("in parsed tasks - found main id task");
                type = task["type"];
                subtasks = {}; // changed from string to int for ease, is this line even needed?
                group_info = task["group_info"].get<std::unordered_map<std::string, int>>();
                group_size = getGroupSize(group_info);

                if (task.contains("area")) {
                    area = task["area"].get<std::unordered_map<std::string, int>>();
                }

                // If action (subtask) didn't have a location defined, pull location from main task
                if (location.first == -1 && location.second == -1 && task.contains("location")) {
                    auto loc_json = task["location"];
                    int x = loc_json.value("x", -1);  // fallback to -1 if missing
                    int y = loc_json.value("y", -1);
                    location = std::make_pair(x, y);
                }

                reward = task["reward"];
            }
        }

        TaskInfo task_struct = {
            id, // int
            name, // string
            type, // string
            prerequisite_failures, // int
            subtasks, // std::vector<int>
            main_id, // int
            group_size, //int
            group_info, // <std::unordered_map<std::string, int>>
            location, // std::pair<int,int>
            area, // <std::unordered_map<std::string, int>>
            reward // double
        };

        all_subtasks_info[id] = task_struct;
    }

    log_info("all_subtasks_info:");
    utils::logAllTasksInfo(all_subtasks_info, *this);

    return all_subtasks_info;
    
}

// TaskInfo& World::getTaskInfo(int task_id) {
//     std::lock_guard<std::mutex> lock(world_mutex);

//     return all_tasks_info.at(task_id);

//     /*if (all_tasks_info.find(task_id) != all_tasks_info.end()) {
//         return all_tasks_info.at(task_id);
//     } else {
//         throw std::runtime_error("Task ID not found in all_tasks_info");
//     }*/
// }

TaskInfo& World::getTaskInfo(int task_id) {

    std::string plz = "Task id in getTaskInfo is: " + std::to_string(task_id);
    log_info(plz);

    std::lock_guard<std::mutex> lock(world_mutex);

    // Check main tasks first
    if (all_tasks_info.count(task_id)) {
        return all_tasks_info.at(task_id);
    }
    
    // Then check subtasks
    if (all_subtasks_info.count(task_id)) {
        return all_subtasks_info.at(task_id);
    }
    
    throw std::runtime_error("Task ID " + std::to_string(task_id) + " not found in all_tasks_info or all_subtasks_info");
}

TaskInfo& World::getSubtaskInfo(int task_id) {
    std::lock_guard<std::mutex> lock(world_mutex);

    return all_subtasks_info.at(task_id);

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

std::vector<int> World::getRobotSubtaskCapabilities(Robot* robot) {

    std::vector<int> doable_local_subtasks;

    std::string agent_type = robot->getType();

    if (all_agent_capabilities.find(agent_type) == all_agent_capabilities.end()) {
        std::cerr << "Error: Capabilities for type " << agent_type << " not found!" << std::endl;
        return {};  // Return an empty vector
    }

    // std::vector<int> robot_capabilities_by_type = all_agent_capabilities[agent_type]; 
    std::unordered_map<std::string,bool> robot_capabilities_by_type = all_agent_capabilities[agent_type]; 

    for (auto& pair : all_subtasks_info) {
        TaskInfo& subtask = pair.second;

        auto it = robot_capabilities_by_type.find(subtask.type);
    
        if (it != robot_capabilities_by_type.end() && it->second == true) {
            // Robot has this capability - found doable subtask
            doable_local_subtasks.push_back(pair.first); // pair.first is task id (int)
        }
        
    }

    // Since we traverse all_subtasks_info which is an unordered map, tasks not necessarily traversed in ascending id order
    // For simplicity, let's sort capabilities so id's are in increasing order
    std::sort(doable_local_subtasks.begin(), doable_local_subtasks.end());
    // utils::log1DVector(doable_local_subtasks, *robot);

    //std::cout << "Doable local tasks vector: " << std::endl;
    // utils::print1DVector(doable_local_subtasks);

    return doable_local_subtasks; // List of specific task indices of all doable types for given robot

}

std::vector<int> World::getRobotCapabilities(Robot* robot) {
    std::lock_guard<std::mutex> lock(world_mutex);

    std::vector<int> doable_local_tasks;

    std::string agent_type = robot->getType();

    // logging to world here stalled it

    if (all_agent_capabilities.find(agent_type) == all_agent_capabilities.end()) {
        std::cerr << "Error: Capabilities for type " << agent_type << " not found!" << std::endl;
        return {};  // Return an empty vector
    }

    std::unordered_map<std::string,bool> robot_capabilities_by_type = all_agent_capabilities[agent_type]; 

    for (auto& pair : all_tasks_info) {
        TaskInfo& local_task = pair.second;

        auto it = robot_capabilities_by_type.find(local_task.type);
    
        if (it != robot_capabilities_by_type.end() && it->second == true) {
            // Robot has this capability - found doable task
            doable_local_tasks.push_back(pair.first); // pair.first is task id (int)
        }
        
    }

    // Since we traverse all_tasks_info which is an unordered map, tasks not necessarily traversed from id 1 and increasing
    // For simplicity, let's sort capabilities so id's are in increasing order
    std::sort(doable_local_tasks.begin(), doable_local_tasks.end());
    // utils::log1DVector(doable_local_tasks, *robot);

    //std::cout << "Doable local tasks vector: " << std::endl;
    // utils::print1DVector(doable_local_tasks);

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

// each ping is a pair of sender id and double timestamp of last cbba/cbga update
/*std::unordered_map<int,std::vector<std::pair<int,double>>>& World::getPingTracker() {
    std::lock_guard<std::mutex> lock(world_mutex);
    return ping_tracker;
}*/

// below only if need all three variables in each ping (id, timestamp of last cbba/cbga update, new self failure flag)
std::unordered_map<int,std::vector<std::tuple<int,double,bool>>>& World::getPingTracker() {
    std::lock_guard<std::mutex> lock(world_mutex);
    return ping_tracker;
}

void World::clear(Pose2D pose) {
    std::lock_guard<std::mutex> lock(world_mutex);
    std::cout << "Clearing world..." << std::endl;
    cv::circle(image, cv::Point(pose.x, pose.y), 5, cv::Scalar(255, 255, 255), -1);
}

void World::startRecording(const std::string& filename, double fps) {
    int codec = 0;  // Uncompressed
    cv::Size frame_size(X, Y);
    
    video_writer.open(filename, codec, fps, frame_size, true);
    
    if (!video_writer.isOpened()) {
        std::cerr << "Failed to open video writer!" << std::endl;
        recording = false;
    } else {
        std::cout << "Recording started" << std::endl;
        recording = true;
    }
}
    
void World::stopRecording() {
    if (recording) {
        video_writer.release();
        recording = false;
        std::cout << "Recording stopped" << std::endl;
    }
}

World::~World() {
    stopRecording();
    std::cout << "World destructor called - recording stopped" << std::endl;
}

void World::plot() {
    // Mutex used to lock whole function but there were big delays for some of the robots

    image = background_image.clone(); // to keep obstacles + anything else that doesn't change in evironment

    std::vector<std::pair<Pose2D, cv::Scalar>> robot_data;
    
    {  // Mutex locking getPose, getColor calls but NOT the time-consuming drawing part (circles get left as strays if don't protect at all)
        std::lock_guard<std::mutex> lock(world_mutex);
        
        // Collect all data while holding lock
        //std::vector<Pose2D> quadrant_centers = getQuadrantCenters(); // don't need this right? it was just to confirm coverage planning visual
        
        for (auto& pair : robot_tracker) {
            Robot* robot = pair.second;
            robot_data.push_back({robot->getPose(), robot->getColor()});
        }
    }  // Lock released here
    
    // Do all the slow drawing/display work without holding lock
    for (auto& data : robot_data) {
        cv::circle(image, cv::Point(data.first.x, data.first.y), 5, data.second, -1);
    }

    if (recording && video_writer.isOpened()) {
        video_writer.write(image);
    }
    
    cv::imshow("World Image", image);
    cv::waitKey(300);  // No longer blocking other robots!
}

void World::trackRobot(Robot* robot) {
    std::lock_guard<std::mutex> lock(world_mutex);
    //std::cout << "Tracking Robot ID: " << robot->getID() << std::endl;
    robot_tracker[robot->getID()] = robot;
}

bool World::inCommsTrue(int id1, int id2) {
    // This is the old inComms, used for world functions that need true list of neighbors in comms 

    std::lock_guard<std::mutex> lock(world_mutex);
    if (robot_tracker.find(id1) == robot_tracker.end() || robot_tracker.find(id2) == robot_tracker.end()) {
        return false;
    }
    Robot* robot1 = robot_tracker[id1];
    Robot* robot2 = robot_tracker[id2];
    Pose2D p1 = robot1->getPose(); Pose2D p2 = robot2->getPose();
    double distance_between_robots = distance->getEuclideanDistance(p1.x,p1.y,p2.x,p2.y);
    return distance_between_robots <= comms_range; // before testing below

}

bool World::inComms(int id1, int id2) {
    std::lock_guard<std::mutex> lock(world_mutex);

    // testing now
    // can log with world log_info here because nested mutex

    if (robot_tracker.find(id1) == robot_tracker.end() || robot_tracker.find(id2) == robot_tracker.end()) {
        return false;
    }
    Robot* robot1 = robot_tracker[id1];
    Robot* robot2 = robot_tracker[id2];
    Pose2D p1 = robot1->getPose(); Pose2D p2 = robot2->getPose();
    double distance_between_robots = distance->getEuclideanDistance(p1.x,p1.y,p2.x,p2.y);

    robot1->log_info("here in incomms 1");
    robot2->log_info("here in incomms 1");

    // First do binary return (either in or out of comms range), corresponds with signal path loss factor of 0 so easy to check with that
    if (decay_rate == 0.0) {
        robot1->log_info("binary comms...");
        robot2->log_info("binary comms...");
        return distance_between_robots <= comms_range;
    } else {

        robot1->log_info("degrading comms...");
        robot2->log_info("degrading comms...");
        // Signal degrades from 1.0 (when distance btwn robots is 0) to ~0 (when distance is ~comms range)
        // double signal_strength = pow(comms_range / std::max(distance_between_robots, 1.0), signal_path_loss_factor);

        // // Normalize to [0,1] range
        // double max_signal = pow(comms_range / 1.0, signal_path_loss_factor);  // Signal at 1 pixel
        // double normalized_signal = std::min(1.0, signal_strength / max_signal);

        // std::string s = "normalized_signal: " + std::to_string(normalized_signal);
        // robot1->log_info(s); robot2->log_info(s);
        // double random_num = dis(gen);
        // std::string s2 = "random_num:" + std::to_string(random_num);
        // robot1->log_info(s2); robot2->log_info(s2);

        // bool comms_success = random_num < normalized_signal;

        double distance_ratio = distance_between_robots / comms_range;
        double success_probability = exp(-decay_rate * distance_ratio);

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);

        std::string s = "success_probability: " + std::to_string(success_probability);
        robot1->log_info(s); robot2->log_info(s);
        double random_num = dis(gen);
        std::string s2 = "random_num:" + std::to_string(random_num);
        robot1->log_info(s2); robot2->log_info(s2);

        bool comms_success = random_num < success_probability;

        std::string s3; std::string s4;
        if (distance_between_robots < comms_range && !comms_success) {
            s3 = "Comms fading resulting in comms failure despite distance less than comms range";
            s4 = "Distance:" + std::to_string(distance_between_robots) + ", comms range: " + std::to_string(comms_range);
            robot1->log_info(s3); robot2->log_info(s3); robot1->log_info(s4); robot2->log_info(s4);
        } else if (distance_between_robots < comms_range && comms_success) {
            std::string s5 = "comms fading on but still comms succeed";
            robot1->log_info(s5); robot2->log_info(s5);
        }
        
    
        return comms_success;

    }
    
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

        // Uncomment to plot
        // cv::rectangle(image, cv::Point(centerA.x - size, centerA.y - size), cv::Point(centerA.x + size, centerA.y + size), cv::Scalar(0, 0, 0), -1);
        // cv::rectangle(image, cv::Point(centerB.x - size, centerB.y - size), cv::Point(centerB.x + size, centerB.y + size), cv::Scalar(0, 0, 0), -1);
        // cv::rectangle(image, cv::Point(centerC.x - size, centerC.y - size), cv::Point(centerC.x + size, centerC.y + size), cv::Scalar(0, 0, 0), -1);
        // cv::rectangle(image, cv::Point(centerD.x - size, centerD.y - size), cv::Point(centerD.x + size, centerD.y + size), cv::Scalar(0, 0, 0), -1);

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
        if ( other_robot_id != robot_id && inCommsTrue(robot_id,other_robot_id)) { // inCommsTrue 
            // Neighbor found! Potential neighbor is not self and is in comms range of robot with id = robot_id so it is a neighbor
            neighbor_ids.push_back(other_robot_id);
        }
    }

    return neighbor_ids;

    //return ping_id_tracker[robot_id]; // old way, but want to avoid pinging for now
}

void World::logNeighbors() {

    log_info("Neighbors now:");
    for (auto& pair : robot_tracker) {
        std::string rbt = "Robot " + std::to_string(pair.first) + ":";
        log_info(rbt);    
        std::vector<int> neighbors = getNeighborsInComms(pair.first);
        utils::log1DVectorFromWorld(neighbors,*this);
    }

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

// bool World::hasTaskInfo(int task_id) {
//     return all_tasks_info.find(task_id) != all_tasks_info.end();
// }

bool World::hasTaskInfo(int task_id) {
    // Updated to account for subtasks
    return all_tasks_info.count(task_id) > 0 || 
           all_subtasks_info.count(task_id) > 0;
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

bool World::fullGroupPresent(int current_task_id) {

    log_info("in fullGroupPresent");
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

void World::updateMessagingLog(int robot_id, int msg_id) {

    messaging_log[robot_id].push_back(msg_id);
}

void World::logMessagingLog() {

    // At time of print to world log, who has heard from whom
    log_info("Logging of sender ids for each msg each robot has received");

    for (auto& pair : robot_tracker) {
        int robot_id = pair.first;
        std::string bla = std::string("Robot ") + std::to_string(robot_id) + ": ";
        log_info(bla);
        utils::log1DVectorFromWorld(messaging_log[robot_id], *this);

    }

}

int World::getPrerequisiteFailureThreshold(std::string subtask_name) {

    for (auto& pair : all_subtasks_info) {
        TaskInfo& subtask = pair.second;

        if (subtask.name == subtask_name) {
            return subtask.prerequisite_failures;
        }
    }

    return -1; // To denote error, failure threshold not found
}

std::unordered_map<int, bool> World::initManualFaultInjectionTracker() {
    auto world_attributes = parser->j["world_attributes"];
    
    // Parse as string keys and int values (what's actually in the JSON)
    auto fault_injection_json = world_attributes["fault_injection"].get<std::unordered_map<std::string, int>>();
    
    // Convert to int keys and bool values
    std::unordered_map<int, bool> fault_injection_tracker;
    for (const auto& [key_str, value] : fault_injection_json) {
        fault_injection_tracker[std::stoi(key_str)] = (value != 0);
    }
    
    return fault_injection_tracker;
}

std::unordered_map<int, bool> World::initFaultInjectionTracker() {

    log_info("in new init for fault injection tracker");

    // Initialize all subtasks with false (no faults)
    for (const auto& [subtask_id, task_info] : all_subtasks_info) {
        fault_injection_tracker[subtask_id] = false;
    }
    injectRandomFaults();

    return fault_injection_tracker;
}

bool World::getFaultInjectionFlag(int task_id) {
    std::lock_guard<std::mutex> lock(world_mutex);

    return fault_injection_tracker[task_id];
}

/*void World::updateFaultInjectionTracker(int task_id, bool fail_flag) {
    std::lock_guard<std::mutex> lock(world_mutex);
    // fail flag 1 if injection defaults failure, 0 when helper has helped do it successfully

    // log_info("in updateFaultInjectionTracker, tracker now: ");
    // utils::logUnorderedMapWorld(fault_injection_tracker, *this);

    fault_injection_tracker[task_id] = fail_flag;

    // log_info("in updateFaultInjectionTracker, tracker after: ");
    // utils::logUnorderedMapWorld(fault_injection_tracker, *this);
}*/

void World::updateFaultInjectionTracker(int task_id, bool fail_flag) {
    log_info("in updateFaultInjectionTracker, about to update");
    utils::logUnorderedMapWorld(fault_injection_tracker, *this);
    {
        std::lock_guard<std::mutex> lock(world_mutex);
        fault_injection_tracker[task_id] = fail_flag;
    }  // Lock released here
    
    log_info("in updateFaultInjectionTracker, tracker updated");
    utils::logUnorderedMapWorld(fault_injection_tracker, *this);
}

int World::getSubtaskID(std::string name) {

    for (auto& pair : all_subtasks_info) {
        TaskInfo& subtask = pair.second;

        if (subtask.name == name) {
            return subtask.id;
        }
    }

    log_info("ERROR: getSubtaskID() - id not found due to name not found");
    return -1;
}

bool World::isSubtaskID(int task_id) {
    log_info("in isSubtaskID");
    std::string hel = "task_id: " + std::to_string(task_id);
    log_info(hel);

    bool test = {std::find(subtask_ids.begin(), subtask_ids.end(), task_id) != subtask_ids.end()};
    std::string p = "issubtaskid: " + std::to_string(test);
    log_info(p);

    return std::find(subtask_ids.begin(), subtask_ids.end(), task_id) != subtask_ids.end();
}

bool World::isLastSubtask(int current_task_id, int local_current_task_id) {

    // testing

    if (isSubtaskID(current_task_id)) {
        return true; // This would be the case for helper mode
    } else {
        // Get subtasks list of current_task_id, which here is main
        TaskInfo& main_task_info = getTaskInfo(current_task_id);
        std::vector<int> subtasks = main_task_info.subtasks;
        int last_subtask_id = subtasks.back();

        if (local_current_task_id == last_subtask_id) {
            return true;
        }

    }

    // Otherwise, is main task but just completed an intermediate subtask so cannot mark main task as complete yet
    return false;

}

void World::initializeBackground() {
    // Draw all obstacles onto the already-created background_image, just once so no redundant expensive plotting
    for (const auto& polygon : obstacles) {
        const cv::Point* pts = polygon.data();
        int npts = polygon.size();
        cv::fillPoly(background_image, &pts, &npts, 1, cv::Scalar(0, 0, 0));
    }

    // for (const auto& blocked_robot_type : unknown_obstacles.keys()) {
    //     for (const auto& polygon : unknown_obstacles[blocked_robot_type]) {
    for (const auto& [blocked_robot_type, obstacles] : unknown_obstacles) {
        for (const auto& polygon : obstacles) {
            const cv::Point* pts = polygon.data();
            int npts = polygon.size();
            cv::fillPoly(background_image, &pts, &npts, 1, cv::Scalar(128, 128, 128));
        }
    }
    
    // Draw X for main tasks
    for (const auto& [task_id, task_info] : all_tasks_info) {
        cv::Point center(task_info.location.first, task_info.location.second);
        int size = 4;
        cv::line(background_image, cv::Point(center.x - size, center.y - size), 
                 cv::Point(center.x + size, center.y + size), cv::Scalar(0, 0, 0), 2);
        cv::line(background_image, cv::Point(center.x - size, center.y + size), 
                 cv::Point(center.x + size, center.y - size), cv::Scalar(0, 0, 0), 2);
    }

    std::cout << "Background with " << obstacles.size() << " obstacles initialized" << std::endl;
}

void World::addObstacle(std::vector<cv::Point> polygon) { // to list, not plotting yet
    obstacles.push_back(polygon);
}

void World::addUnknownObstacle(std::string blocked_robot_type, std::vector<cv::Point> polygon) { // to list, not plotting yet
    unknown_obstacles[blocked_robot_type].push_back(polygon);
}

double World::initSignalPathLossFactor() {

    double signal_path_loss_factor = 0;

    auto world_attrs = parser->j["world_attributes"];

    if (world_attrs.contains("signal_path_loss_factor")) {
        signal_path_loss_factor = world_attrs["signal_path_loss_factor"];
    }

    return signal_path_loss_factor;
}

double World::initDecayRate() {

    double decay_rate = 0;

    auto world_attrs = parser->j["world_attributes"];

    if (world_attrs.contains("decay_rate")) {
        decay_rate = world_attrs["decay_rate"];
    }

    return decay_rate;
}

void World::getObstacles() {
    // Parse obstacles

    auto world_attrs = parser->j["world_attributes"];
    
    if (world_attrs.contains("obstacles")) {
        for (const auto& obstacle_json : world_attrs["obstacles"]) {
            std::vector<cv::Point> polygon;
            
            for (const auto& point : obstacle_json["points"]) {
                int x = point["x"].get<int>();
                int y = point["y"].get<int>();
                polygon.push_back(cv::Point(x, y));
            }
            
            addObstacle(polygon);
        }
    }
    
    std::cout << "Loaded " << obstacles.size() << " obstacles from JSON" << std::endl;
}

void World::getUnknownObstacles() {
    // Parse unknown obstacles

    auto world_attrs = parser->j["world_attributes"];
    
    if (world_attrs.contains("unknown_obstacles")) {
        for (const auto& obstacle_json : world_attrs["unknown_obstacles"]) {
            std::vector<cv::Point> polygon;

            for (const auto& blocked_robot_type : obstacle_json["blocks"]) {
                for (const auto& point : obstacle_json["points"]) {
                    int x = point["x"].get<int>();
                    int y = point["y"].get<int>();
                    polygon.push_back(cv::Point(x, y));
                }
            
                addUnknownObstacle(blocked_robot_type, polygon);
            }
            
            
        }
    }
    
    std::cout << "Loaded " << unknown_obstacles.size() << " unknown obstacles from JSON" << std::endl;
}

bool World::isObstacle(int x, int y) {
    cv::Point test_point(x, y);
    
    for (const auto& polygon : obstacles) {

        // double result = cv::pointPolygonTest(polygon, test_point, false); // positive if in polygone, negative outside, 0 on edge
        
        // if (result >= 0) {  // Inside or on edge = obstacle
        //     return true;
        // }

        double distance = cv::pointPolygonTest(polygon, test_point, true);  // true = measure distance
        
        // If inside polygon OR within robot radius (5)
        if (distance >= -5) {
            return true;
        }
    }
    
    return false;  // Not in any obstacle
}

std::pair<bool, std::vector<cv::Point>> World::isUnknownObstacle(Pose2D waypoint, std::string robot_type) {
    std::pair<bool, std::vector<cv::Point>> is_obstacle = {false, std::vector<cv::Point>()};
    
    cv::Point test_point(waypoint.x, waypoint.y);
    
    std::string wp = "Checking waypoint (" + std::to_string(waypoint.x) + ", " + std::to_string(waypoint.y) + ") for robot type: " + robot_type;
    log_info(wp);
    
    // Check if this robot type has unknown obstacles
    if (unknown_obstacles.find(robot_type) == unknown_obstacles.end()) {
        log_info("No unknown obstacles defined for this robot type");
        return is_obstacle;  // No unknown obstacles for this robot type
    }
    
    std::string count = "Found " + std::to_string(unknown_obstacles[robot_type].size()) + " unknown obstacles for " + robot_type;
    log_info(count);
    
    // Check each unknown obstacle polygon for this robot type
    for (const auto& polygon : unknown_obstacles[robot_type]) {
        double distance = cv::pointPolygonTest(polygon, test_point, true);  // true = measure distance
        
        std::string dist_msg = "Distance to obstacle: " + std::to_string(distance);
        log_info(dist_msg);
        
        // If inside polygon OR within robot radius (5)
        if (distance >= -5) {
            log_info("WAYPOINT IS IN UNKNOWN OBSTACLE!");
            is_obstacle.first = true;
            is_obstacle.second = polygon;  // Return the polygon that blocks
            return is_obstacle;
        }
    }
    
    log_info("Waypoint is clear of unknown obstacles");
    return is_obstacle;  // Not in any unknown obstacle
}

void World::injectRandomFaults() {

    log_info("in injectRandomFaults for fault_percentage: ");
    std::string b = std::to_string(fault_percentage);
    log_info(b);
    
    std::mt19937 gen;
    if (random_seed == 0) { // No seed given
        std::random_device rd;
        gen.seed(rd());
    } else { // seed given, repeatable randomness
        gen.seed(random_seed);
    }

    std::uniform_real_distribution<> dis(0.0, 1.0);
    
    // This method initializes each action with a given % chance of failure
    for (auto& [subtask_id, fault_flag] : fault_injection_tracker) {
        // Roll dice: if under percentage, inject fault (true), otherwise no fault (false)
        fault_flag = (dis(gen) < fault_percentage);
    }
}

double World::initFaultInjectionPercentage() {
    double fault_percentage = 0; // no faults

    auto world_attrs = parser->j["world_attributes"];

    if (world_attrs.contains("fault_percentage")) {
        fault_percentage = world_attrs["fault_percentage"].get<double>() / 100.0;
    }

    return fault_percentage;
}

double World::initRandomSeed() {
    double random_seed = 0; // 0 because why not

    auto world_attrs = parser->j["world_attributes"];

    if (world_attrs.contains("random_seed")) {
        random_seed = world_attrs["random_seed"];
    }

    return random_seed;
}

/*void World::updateCumulativeDiscountedReward(double reward) {
    std::lock_guard<std::mutex> lock(world_mutex);
    cumulative_discounted_reward += reward;
}*/

void World::updateCumulativeScore(double score) {
    std::lock_guard<std::mutex> lock(world_mutex);
    cumulative_score += score;
}