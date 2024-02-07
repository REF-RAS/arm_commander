#ifndef COMMANDER_H
#define COMMANDER_H

#include <functional>
#include <memory>
#include <string>
#include <chrono>
#include <iostream>
#include <sstream>
#include <rclcpp/rclcpp.hpp>

typedef enum 
{ 
    INFO, 
    ERROR, 
    WARN, 
    DEBUG 
} LOG_LEVEL;

class Commander
{
private:
    /// @brief Class-level public logging method for ease of use in ROS
    /// @param txt 
    /// @param level 
    void _logger(std::string txt, LOG_LEVEL level = LOG_LEVEL::INFO);

    const std::string _name = "arm_commander";
    std::string _group_name;
    std::string _world_link;
    
public:
    /// @brief Constructor for Commander Class
    Commander(std::string _group_name, std::string world_link);
    /// @brief Destructor for Commander Class
    ~Commander();

    


};


#endif // COMMANDER_H