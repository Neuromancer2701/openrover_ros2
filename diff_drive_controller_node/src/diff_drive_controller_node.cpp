#include "diff_drive_controller_node/diff_drive_controller_node.hpp"


#include <algorithm>
#include <unordered_map>
#include <chrono>


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

    vesc_interface_ = std::make_unique<vesc::Vesc>();
    vesc_interface_->FindandMapMotorControllers();

    if(!vesc_interface_->isTwoWheelDrive())
    {
        RCLCPP_INFO(get_logger(), "Failed to find at least two wheels.");
        return CallbackReturn::ERROR;
    }

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

    // Create and start the timer (e.g., at 100 Hz)
    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&DiffDriveController::timerCallback, this)
    );

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

    auto left_rpm  = std::max(-MAX_RPM_LIMIT, std::min(static_cast<int>(left_wheel_velocity * rads_sec_to_RPM),MAX_RPM_LIMIT));
    auto right_rpm = std::max(-MAX_RPM_LIMIT, std::min(static_cast<int>(right_wheel_velocity * rads_sec_to_RPM),MAX_RPM_LIMIT));

    std::map<int, int> target_rpms{{vesc::Vesc::wheel_ids::left_back,left_rpm},
                                   {vesc::Vesc::wheel_ids::right_back,right_rpm}};

    set_target_rpms(target_rpms);
}

// Helper function to publish wheel velocities
void DiffDriveController::publishWheelVelocity()
{
    if (!left_wheel_pub_->is_activated() || !right_wheel_pub_->is_activated())
    {
        RCLCPP_WARN(get_logger(), "Trying to publish while the node is not activated.");
        return;
    }

    double left_velocity{0.0};
    double right_velocity{0.0};
    {
        std::lock_guard<std::mutex> lock(motor_data_mutex_);
        left_velocity = motor_map_[vesc::Vesc::wheel_ids::left_back].rpm;
        right_velocity = motor_map_[vesc::Vesc::wheel_ids::right_back].rpm;
    }

    auto left_wheel_msg = std_msgs::msg::Float64();
    left_wheel_msg.data = left_velocity;
    left_wheel_pub_->publish(left_wheel_msg);

    auto right_wheel_msg = std_msgs::msg::Float64();
    right_wheel_msg.data = right_velocity;
    right_wheel_pub_->publish(right_wheel_msg);
}

void DiffDriveController::timerCallback()
{
    RCLCPP_INFO(get_logger(), "Read Motor data.");
    //Read motor data
    read_hardware();

    RCLCPP_INFO(get_logger(), "Write Motor RPMs");
    // Write motor RPMs
    write_hardware();

    // Publish wheel velocities
    publishWheelVelocity();
}

void DiffDriveController::read_hardware()
{
    std::lock_guard<std::mutex> lock(motor_data_mutex_);
    auto incoming_data = vesc_interface_->GetAllMotorData();
    for(const auto&[id, data]:incoming_data)
    {
        motor_data temp;
        temp.volt_in = data.v_in;
        temp.temp_motor = data.temp_motor;
        temp.current_motor = data.current_motor;
        temp.tachometer_abs = data.tachometer_abs;
        temp.fault_code = data.fault_code;
        temp.fault_str = data.fault_str;
        temp.rpm = data.rpm;

        motor_map_[id] = temp;
    }
}
void DiffDriveController::write_hardware()
{
    std::lock_guard<std::mutex> lock(motor_data_mutex_);

    std::unordered_map<int, int> wheel_rpms;
    for(const auto&[id, data]:motor_map_)
    {
        wheel_rpms.try_emplace(id, data.target_rpm);
    }

    vesc_interface_->SetWheelsRPM(wheel_rpms);
}

void DiffDriveController::set_target_rpms(const std::map<int, int>& target_rpms)
{
    std::lock_guard<std::mutex> lock(motor_data_mutex_);
    for(const auto&[id, rpm]:target_rpms)
    {
        motor_map_[id].target_rpm = rpm;
    }
}

}// namespace openrover::base_control

