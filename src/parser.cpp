#include "parser.hpp"
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <unordered_set>
#include <unordered_map>
#include <utils.hpp>

using json = nlohmann::json;

JSONParser::JSONParser(const std::string& input_filepath) {
    input_path = input_filepath;
    j = parse();

}

json JSONParser::parse() {

    // Get JSON object j
    std::ifstream file(input_path);
    json j;

    try {
        file >> j;
    } catch (json::parse_error& e) {
        std::cerr << "JSON Parse Error: " << e.what() << std::endl;
    }
    return j;
}

int JSONParser::getNumAgents() {

    int num_agents;

    if (j.contains("agents") && j["agents"].is_array()) {
        num_agents = j["agents"].size();
        std::cout << "Number of agents: " << num_agents << std::endl;
    }
    else {
        std::cerr << "Error: agent info not found or invalid in JSON." << std::endl;
    }

    return num_agents;
}

int JSONParser::getNumLocalTasks() {

    int num_local_tasks;

    if (j.contains("local_tasks") && j["local_tasks"].is_array()) {
        num_local_tasks = j["local_tasks"].size();
        std::cout << "Number of local tasks: " << num_local_tasks << std::endl;
    }

    return num_local_tasks;
}

int JSONParser::getMaxDepth() {

    int max_depth;

    if (j.contains("cbba") && j["cbba"].is_object()) {
        if (j["cbba"].contains("max_depth") && j["cbba"]["max_depth"].is_number_integer()) {
            //std::cout << j["cbba"]["max_depth"] << std::endl;
            max_depth = j["cbba"]["max_depth"];
        } else {
            std::cerr << "Error: max_depth not found or invalid in JSON." << std::endl;
        }
    }
    else {
        std::cerr << "Error: cbba section not found in JSON." << std::endl;
    }

    return max_depth;
}

std::vector<int> JSONParser::getAgentIndices() {

    std::vector<int> agent_indices;

    if (j.contains("agents") && j["agents"].is_array()) {
        for (const auto& agent : j["agents"]) {
            agent_indices.push_back(agent["id"]);
            std::cout << agent["id"] << std::endl;
        }
    }
    else {
        std::cerr << "Error: agent indices not found or invalid in JSON." << std::endl;
    }

    return agent_indices;
}

std::vector<std::string> JSONParser::getAgentTypes() {

    std::unordered_set<std::string> unique_agent_types; // unordered set to only get unique types

    if (j.contains("agents") && j["agents"].is_array()) {
        for (const auto& agent : j["agents"]) {
            unique_agent_types.insert(agent["type"].get<std::string>()); // Have to explicitly convert to string, json library can't do unless basic like vector
        }
    }
    else {
        std::cerr << "Error: agent types not found or invalid in JSON." << std::endl;
    }

    std::vector<std::string> agent_types(unique_agent_types.begin(), unique_agent_types.end());

    return agent_types;
}

std::vector<std::string> JSONParser::getTaskTypes() {

    std::unordered_set<std::string> unique_task_types; // unordered set to only get unique types

    if (j.contains("local_tasks") && j["local_tasks"].is_array()) {
        for (const auto& task : j["local_tasks"]) {
            unique_task_types.insert(task["type"].get<std::string>()); // Have to explicitly convert to string, json library can't do unless basic like vector
        }
    }
    else {
        std::cerr << "Error: task types not found or invalid in JSON." << std::endl;
    }

    std::vector<std::string> task_types(unique_task_types.begin(), unique_task_types.end());

    return task_types;
}

std::unordered_map<std::string, std::vector<int>> JSONParser::getAgentCapabilities(std::vector<std::string> agent_types, std::vector<std::string> task_types) {

    // Not including checks for JSON agents or local_tasks array existence
    // This function is called in CBBA init AFTER functions with those checks run successfully

    // Note that the order of the rows (agent types) will of course match the agent_types vector,
    // and not necessarily the order in the JSON agent_capabilities

    std::unordered_map<std::string, std::vector<int>> capabilities;

    //std::cout << "Creating capabilities map" << std::endl;
    //std::cin.get();

    //for (int i = 0; i < agent_types.size(); i++) {
    for (auto agent_type : agent_types) {
        std::vector<int> row; // Create a row for each agent type
        for (int j = 0; j < task_types.size(); j++) { // Each column is different task type
            //std::cout << task_types[j] << std::endl;
            row.push_back(getCompatibility(agent_type, task_types[j]));
        }
        //std::cout << "Row for " << agent_type << " is " << std::endl;
        //utils::print1DVector(row);
        capabilities[agent_type] = row; // Add capabilities vector 
        //std::cout << "Capabilities found for " << type << ": ";
    }

    //std::cout << "Agent capabilities retrieved, size: " << capabilities.size() << std::endl;


    //std::cout << "End in parser capabilities function" << std::endl;
    //std::cin.get();

    return capabilities;
}

int JSONParser::getCompatibility(std::string agent_type, std::string task_type) {

    // 0 - not able; 1 - able; 
    // Dealing with these later: 2 - able with assistance (co-op); 3 - co-op after fault

    if (j.contains("agent_capabilities") && j["agent_capabilities"].is_array()) {
        //std::cout << agent_type << std::endl;
        //std::cout << task_type << std::endl;
        //std::cin.get();
        for (const auto& t : j["agent_capabilities"][1][agent_type]) {
            //std::cout << t << " " << task_type << std::endl;
            //std::cin.get();
            if (t == task_type) {
                //std::cout << "agent type for found match: " << agent_type << " 1" << std::endl;
                return 1;
            }
        }
    }
    else {
        std::cerr << "Error: agent_capabilities not found or invalid in JSON." << std::endl;  
    }

    //std::cout << "agent type for found match: " << agent_type << " 0" << std::endl;
    return 0; // Did not find given task type in list of capabilities (task types it can do) for given agent type
}

/*json JSONParser::getAgentsList() {

}*/