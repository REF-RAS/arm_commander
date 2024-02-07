#ifndef COMMANDER_H
#define COMMANDER_H

#include <functional>
#include <memory>
#include <string>
#include <chrono>
#include <iostream>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

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
    std::string _planning_group;
    std::string _world_link;
    rclcpp::Node::SharedPtr _node;
    moveit::core::RobotModelPtr _robot_model;
    moveit::core::RobotStatePtr _robot_state;
    robot_model_loader::RobotModelLoader _robot_model_loader;
    moveit::planning_interface::MoveGroupInterface _move_group_interface;
    // moveit::planning_interface::PlanningSceneInterface _planning_scene_interface;
public:
    /// @brief Constructor for Commander Class
    explicit Commander(std::string planning_group, std::string world_link, rclcpp::Node::SharedPtr node);
    /// @brief Destructor for Commander Class
    ~Commander();

    


};


#endif // COMMANDER_H