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

};



#endif // PARSE_JSON_H