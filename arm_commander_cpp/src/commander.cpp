#include "commander.h"

// --------------- PUBLIC METHODS -------------------------
Commander::Commander(std::string group_name, std::string world_link)
{
    this->_logger("Constructing!", LOG_LEVEL::INFO);

    _group_name = group_name;
    _world_link = world_link;

    std::ostringstream oss;
    oss << "Group Name: " << _group_name << " | World Link: " << _world_link;
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