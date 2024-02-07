#include "commander.h"

// --------------- PUBLIC METHODS -------------------------
Commander::Commander(std::string planning_group, std::string world_link, rclcpp::Node::SharedPtr node) : 
                                    _robot_model_loader(node, "robot_description"),
                                    _move_group_interface(node, planning_group)

{
    // Initialise variables
    _node = node;
    _planning_group = planning_group;
    _world_link = world_link;
    this->_logger("Constructed", LOG_LEVEL::INFO);
    std::ostringstream oss;
    oss << "Planning group: " << _planning_group << " | World Link: " << _world_link;
    this->_logger(oss.str(), LOG_LEVEL::INFO);
}

Commander::~Commander()
{

}

// -------------- PRIVATE METHODS ---------------------------
void Commander::_logger(std::string txt, LOG_LEVEL level)
{
    switch (level)
    {
    case LOG_LEVEL::INFO:
        RCLCPP_INFO(rclcpp::get_logger(this->_name), txt.c_str());
        break;
    case LOG_LEVEL::ERROR:
        RCLCPP_ERROR(rclcpp::get_logger(this->_name), txt.c_str());
        break;
    default:
        RCLCPP_ERROR(rclcpp::get_logger(this->_name), "UNKNOWN LOG ERROR");
        break;
    }
}