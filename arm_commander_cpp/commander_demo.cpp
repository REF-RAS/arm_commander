#include "commander.h"

class CommanderDemo : public rclcpp::Node
{
    public:
        /// @brief Constructor
        CommanderDemo(): Node("commander_demo")
        {
            // Do Nothing           
            logger("CONSTRUCTING DEMO", LOG_LEVEL::INFO); 

            // On construction run demo
            runDemo();
        }

        /// @brief Main demonstration method
        /// @param void
        void runDemo(void)
        {
            logger("RUNNING DEMO", LOG_LEVEL::INFO); 

            // Create a dynamic commader object
            Commander* arm_commander = new Commander("manipulator", "base_link");
            
            

        }

        /// @brief Class-level public logging method for ease of use in ROS
        /// @param txt 
        /// @param level 
        void logger(std::string txt, LOG_LEVEL level = LOG_LEVEL::INFO)
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
                break;
            }
        }

    private:
        /// @brief Basic Node Name
        const std::string _name = "commander_demo";
};

/// @brief Main Function
/// @param argc 
/// @param argv 
/// @return 
int main(int argc, char * argv[])
{
    // Initialise and create the node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommanderDemo>());
    rclcpp::shutdown();
    return 0;
}