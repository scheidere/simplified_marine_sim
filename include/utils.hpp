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


// Replaced printCapabilities
template <typename T>
void logMapOfVectors(const std::unordered_map<std::string, std::vector<T>>& map_data, Robot& robot) {
    std::ostringstream log_msg;
    for (const auto& pair : map_data) {
        log_msg << pair.first << " : [";
        for (size_t i = 0; i < pair.second.size(); ++i) {
            log_msg << pair.second[i];
            if (i < pair.second.size() - 1) {
                log_msg << ", ";
            }
        }
        log_msg << "]\n";
    }
    robot.log_info(log_msg.str());
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

// Log unordered_map with Pose2D values  
template <>
inline void logUnorderedMap<int, Pose2D>(const std::unordered_map<int, Pose2D>& my_map, Robot& robot) {
    std::ostringstream log_msg;
    for (const auto& pair : my_map) {
        log_msg << pair.first << " : (" << pair.second.x << "," 
                << pair.second.y << "," << pair.second.theta << ")\n";
    }
    robot.log_info(log_msg.str());
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

// Log 1D vector of std::pair<int,double> elements
template <>
inline void log1DVector<std::pair<int,double>>(const std::vector<std::pair<int,double>>& vec, Robot& robot) {
    std::ostringstream log_msg;
    for (const auto& elem : vec) {
        log_msg << "(" << elem.first << "," << elem.second << ") ";
    }
    robot.log_info(log_msg.str());
}

// Log 1D vector of std::pair<int,Pose2D> elements
template <>
inline void log1DVector<std::pair<int,Pose2D>>(const std::vector<std::pair<int,Pose2D>>& vec, Robot& robot) {
    std::ostringstream log_msg;
    for (const auto& elem : vec) {
        log_msg << "(" << elem.first << ",(" << elem.second.x << "," << elem.second.y << "," << elem.second.theta << ")) ";
    }
    robot.log_info(log_msg.str());
}

// Log a 2D vector with rows on separate lines
template <typename T>
void log2DVector(const std::vector<std::vector<T>>& vec, Robot& robot) {
    std::ostringstream log_msg;
    for (size_t i = 0; i < vec.size(); ++i) {
        for (const auto& elem : vec[i]) {
            log_msg << elem << " ";
        }
        log_msg << "\n";
    }
    robot.log_info(log_msg.str());
}

inline void logMsgVector(const std::vector<Msg>& msgs, Robot& robot) {
    std::ostringstream log_msg;
    log_msg << "Messages: ";
    for (const auto& msg : msgs) {
        log_msg << "ID:" << msg.id << " ";
    }
    robot.log_info(log_msg.str());
}

} // namespace end


#endif  // UTILS_HPP
