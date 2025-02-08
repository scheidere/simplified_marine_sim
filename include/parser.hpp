#ifndef PARSE_JSON_H
#define PARSE_JSON_H

#include <nlohmann/json.hpp>

using json = nlohmann::json;

class Parser {
private:
	json input_file;

public:
	Parser(const std::string& input_path);
	json parse(const std::string& input_path);

	// Function to parse the JSON file and do something with the data
	void parseJSON(const std::string& filename);

};



#endif // PARSE_JSON_H