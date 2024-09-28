#include "diff_driver_controller_node/diff_driver_controller_node.hpp"

#include <memory>


int main(int argc, char **argv)
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    // Create and run the lifecycle node
    auto node = std::make_shared<openrover::base_control::DiffDriveController>(options);
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node->get_node_base_interface());

    // Run the executor
    exec.spin();

    // Clean up and shutdown
    rclcpp::shutdown();
    return 0;
}