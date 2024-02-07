#include "commander.h"

class Demo : public rclcpp::Node
{
    public:
        Demo() : rclcpp::Node("commander_demo")
        {
            // --- Create the arm commander
            // Pass in node handle as shared object
            Commander arm_commander("manipulator", "base_link", std::make_shared<rclcpp::Node>(this->get_name()));
        }

        void runDemo(void)
        {
            
        }
};

/// @brief Main Function
/// @param argc 
/// @param argv 
/// @return 
int main(int argc, char * argv[])
{
    // Initialise and create the node
    rclcpp::init(argc, argv);

    auto demo = std::make_shared<Demo>();

    rclcpp::spin(demo);
    rclcpp::shutdown();
    return 0;

    // rclcpp::NodeOptions node_options;
    // node_options.automatically_declare_parameters_from_overrides(true);
    // auto node = rclcpp::Node::make_shared("commander_demo", node_options);

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // // about the robot's state.
    // rclcpp::executors::SingleThreadedExecutor executor;
    // executor.add_node(move_group_node);
    // std::thread([&executor]() { executor.spin(); }).detach();

    // // Start Test
    // Commander arm_commander("manipulator", "base_link", node);

    // --- Reached End
    // rclcpp::shutdown();
    // return 0;
}