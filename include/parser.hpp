#ifndef PARSE_JSON_H
#define PARSE_JSON_H

#include <nlohmann/json.hpp>

using json = nlohmann::json;

class JSONParser {
private:
	std::string input_path;

public:
	JSONParser(const std::string& input_path);

	json j; // Making this public for ease

	json parse();

	int getNumAgents();

	int getNumLocalTasks();

	int getNumSubtasks();

	int getMaxDepth();

	int getConvergenceThreshold();

	std::vector<int> getAgentIndices();

	std::vector<std::string> getAgentTypes();

	std::vector<std::string> getTaskTypes();

	// std::unordered_map<std::string, std::vector<int>> getAgentCapabilities(std::vector<std::string> agent_types, std::vector<std::string> task_types);
	std::unordered_map<std::string, std::unordered_map<std::string,bool>> getAgentCapabilities(std::vector<std::string> agent_types, std::vector<std::string> task_types);

	int getCompatibility(std::string agent_type, std::string task_type);

	//json getAgentsList();

};



#endif // PARSE_JSON_H