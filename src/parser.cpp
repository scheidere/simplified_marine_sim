#include "parser.hpp"
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

Parser::Parser(const std::string& input_path) {

}

json Parser::parse(const std::string& input_path) {

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

void Parser::parseJSON(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        std::cin.get();
        return;
    }

    json j;
    try {
        file >> j;
    } catch (json::parse_error& e) {
        std::cerr << "JSON Parse Error: " << e.what() << std::endl;
        return;
    }

    std::cout << "+++++++++++++++++++++" << std::endl;
    std::cout << "All input shown below: " << j.dump(4) << std::endl;
    std::cout << "+++++++++++++++++++++" << std::endl;

    std::cout << "Parsed global tasks: " << std::endl;
    json global_tasks = j["global_tasks"];
    std::cout << global_tasks.dump(4) << std::endl;
    for (auto& task : global_tasks) {
        std::cout << "ID: " << task["id"] << std::endl;
    }
    //std::cout << "ID: " << global_tasks[0]<< std::endl;
    std::cout << "=====================" << std::endl;

    std::cout << "Parsed JSON successfully.\nPress Enter to continue...";
    std::cin.get();
}