#include "diff_driver_controller_node/diff_driver_controller_node.hpp"

namespace openrover::base_control
{

DiffDriveController::DiffDriveController(const rclcpp::NodeOptions & node_options)
: rclcpp_lifecycle::LifecycleNode("diff_drive_controller", node_options)
{
    // Declare parameters
    this->declare_parameter<double>("wheel_separation", 0.5);
    this->declare_parameter<double>("wheel_radius", 0.1);
}

// Lifecycle callback: on_configure
CallbackReturn DiffDriveController::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Configuring...");

    // Get parameters
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();

    // Publishers
    left_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64>("left_wheel_velocity", 10);
    right_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64>("right_wheel_velocity", 10);

    RCLCPP_INFO(get_logger(), "Configuration complete.");
    return CallbackReturn::SUCCESS;
}

// Lifecycle callback: on_activate
CallbackReturn DiffDriveController::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Activating...");

    // Activate publishers
    left_wheel_pub_->on_activate();
    right_wheel_pub_->on_activate();

    // Subscriber to cmd_vel
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&DiffDriveController::cmdVelCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Node activated.");
    return CallbackReturn::SUCCESS;
}

// Lifecycle callback: on_deactivate
CallbackReturn DiffDriveController::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Deactivating...");

    // Deactivate publishers
    left_wheel_pub_->on_deactivate();
    right_wheel_pub_->on_deactivate();

    // Reset subscriber
    cmd_vel_sub_.reset();

    RCLCPP_INFO(get_logger(), "Node deactivated.");
    return CallbackReturn::SUCCESS;
}

// Lifecycle callback: on_cleanup
CallbackReturn DiffDriveController::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning up...");

    // Cleanup publishers
    left_wheel_pub_.reset();
    right_wheel_pub_.reset();

    // Cleanup subscriber
    cmd_vel_sub_.reset();

    RCLCPP_INFO(get_logger(), "Node cleaned up.");
    return CallbackReturn::SUCCESS;
}

// Lifecycle callback: on_shutdown
CallbackReturn DiffDriveController::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Shutting down...");
    return CallbackReturn::SUCCESS;
}

// Lifecycle callback: on_error
CallbackReturn DiffDriveController::on_error(const rclcpp_lifecycle::State &)
{
    RCLCPP_ERROR(get_logger(), "An error occurred.");
    return CallbackReturn::ERROR;
}

// Callback for cmd_vel
void DiffDriveController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Extract linear and angular velocities
    double linear_velocity = msg->linear.x;
    double angular_velocity = msg->angular.z;

    // Compute wheel velocities using differential drive kinematics
    double left_wheel_velocity = (linear_velocity - angular_velocity * wheel_separation_ / 2.0) / wheel_radius_;
    double right_wheel_velocity = (linear_velocity + angular_velocity * wheel_separation_ / 2.0) / wheel_radius_;

    // Publish wheel velocities
    publishWheelVelocity(left_wheel_velocity, right_wheel_velocity);
}

// Helper function to publish wheel velocities
void DiffDriveController::publishWheelVelocity(double left_velocity, double right_velocity)
{
    if (!left_wheel_pub_->is_activated() || !right_wheel_pub_->is_activated())
    {
        RCLCPP_WARN(get_logger(), "Trying to publish while the node is not activated.");
        return;
    }

    auto left_wheel_msg = std_msgs::msg::Float64();
    left_wheel_msg.data = left_velocity;
    left_wheel_pub_->publish(left_wheel_msg);

    auto right_wheel_msg = std_msgs::msg::Float64();
    right_wheel_msg.data = right_velocity;
    right_wheel_pub_->publish(right_wheel_msg);
}
}// namespace openrover::base_control

