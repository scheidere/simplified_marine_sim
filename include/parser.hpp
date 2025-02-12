#ifndef PARSE_JSON_H
#define PARSE_JSON_H

#include <nlohmann/json.hpp>

using json = nlohmann::json;

class JSONParser {
private:
	std::string input_path;
	json j;

public:
	JSONParser(const std::string& input_path);

	json parse();

	int getNumAgents();

	int getNumLocalTasks();

	int getMaxDepth();

	std::vector<int> getAgentIndices();

	std::vector<std::string> getAgentTypes();

	std::vector<std::string> getTaskTypes();

	std::vector<std::vector<int>> getAgentCapabilities(std::vector<std::string> agent_types, std::vector<std::string> task_types);

	int getCompatibility(std::string agent_type, std::string task_type);

};



#endif // PARSE_JSON_H