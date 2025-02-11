#include "parser.hpp"
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <unordered_set>


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