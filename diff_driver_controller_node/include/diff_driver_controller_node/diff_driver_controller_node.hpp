#ifndef DIFF_DRIVE_CONTROLLER_NODE_HPP
#define DIFF_DRIVE_CONTROLLER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>


namespace openrover::base_control
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/*!
 * @brief Node to provide Diff Drive Kinematics
 */
class DiffDriveController : public rclcpp_lifecycle::LifecycleNode {
public:
    DiffDriveController(const rclcpp::NodeOptions & node_options);

private:
/*!
 * @brief Callback for the cmd_vel subscriber
 */
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

/*!
 * @brief Helper function to publish wheel velocities
 */
    void publishWheelVelocity(double left_velocity, double right_velocity);

 /*!
  * @brief Lifecycle callbacks
  */
    CallbackReturn on_configure(const rclcpp_lifecycle::State &);

    CallbackReturn on_activate(const rclcpp_lifecycle::State &);

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

    CallbackReturn on_error(const rclcpp_lifecycle::State &);

 /*!
  * @brief  Subscriptions and Publishers
  */
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr left_wheel_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr right_wheel_pub_;

 /*!
  * @brief Robot parameters in meters
  */
    double wheel_separation_;
    double wheel_radius_;
};

}// namespace openrover::base_control
#endif // DIFF_DRIVE_CONTROLLER_LIFECYCLE_HPP
