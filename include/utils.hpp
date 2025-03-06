#ifndef UTILS_HPP
#define UTILS_HPP

#include <vector>
#include <iostream>
#include <variant>
#include <robot.hpp>

namespace utils {

// Print a 1D vector
template <typename T>
void print1DVector(const std::vector<T>& vec) {
    for (const auto& elem : vec) {
        std::cout << elem << " ";
    }
    std::cout << std::endl;
}

// Print a 2D vector
template <typename T>
void print2DVector(const std::vector<std::vector<T>>& vec) {
    for (const auto& row : vec) {
        for (const auto& elem : row) {
            std::cout << elem << " ";
        }
        std::cout << std::endl;
    }
}

// TODO: generalize this to print more generally without reference to "agent capabilities"
template <typename T>
void printCapabilities(const std::unordered_map<std::string, std::vector<T>>& all_agent_capabilities) {
    std::cout << "Printing all_agent_capabilities:\n";
    for (const auto& pair : all_agent_capabilities) {
        std::cout << "Robot Type: " << pair.first << " -> Capabilities: ";
        for (const auto& capability : pair.second) {
            std::cout << capability << " ";
        }
        std::cout << std::endl;
    }
}


// Template function to print an unordered_map with any value type
template <typename K, typename V>
void printUnorderedMap(const std::unordered_map<K, V>& my_map) {
    for (const auto& pair : my_map) {
        std::cout << pair.first << " : " << pair.second << std::endl;
    }
}

// Log a map
template <typename K, typename V>
void logMap(const std::map<K, V>& my_map, Robot& robot) {
    std::ostringstream log_msg;
    for (const auto& pair : my_map) {
        log_msg << pair.first << " : " << pair.second << "\n";
    }
    robot.log_info(log_msg.str()); // robot function
}

// Log an unordered_map
template <typename K, typename V>
void logUnorderedMap(const std::unordered_map<K, V>& my_map, Robot& robot) {
    std::ostringstream log_msg;
    for (const auto& pair : my_map) {
        log_msg << pair.first << " : " << pair.second << "\n";
    }
    robot.log_info(log_msg.str()); // robot function
}

// Log a 1D vector
template <typename T>
void log1DVector(const std::vector<T>& vec, Robot& robot) {
    std::ostringstream log_msg;
    for (const auto& elem : vec) {
        log_msg << elem << " ";
    }
    robot.log_info(log_msg.str()); // robot function
}

} // namespace end


#endif  // UTILS_HPP
