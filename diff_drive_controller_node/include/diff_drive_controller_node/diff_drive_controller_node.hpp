#ifndef DIFF_DRIVE_CONTROLLER_NODE_HPP
#define DIFF_DRIVE_CONTROLLER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include "libvesc/Vesc.h"

#include <memory>
#include <map>
#include <string>
#include <mutex>

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
    void publishWheelVelocity();

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
 * @brief  Timer callback
 */
    void timerCallback();

/*!
 * @brief Read Data from Hardware
 */
   void read_hardware();

/*!
* @brief Write RPM to Hardware
*/
   void write_hardware();

 /*!
  * @brief  Subscriptions and Publishers
  */
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr left_wheel_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr right_wheel_pub_;

/*!
 * @brief  Timer for periodically communicating data to and from the vesc controllers.
 */
    rclcpp::TimerBase::SharedPtr timer_;

 /*!
  * @brief Robot parameters in meters
  */
    double wheel_separation_;
    double wheel_radius_;

/*!
 * @brief Motor data(data read and target rpm)
 */
    struct motor_data
    {
        double volt_in;
        double temp_motor;
        double current_motor;
        int tachometer_abs;
        int fault_code;
        std::string fault_str;
        double rpm;
        int target_rpm;
    };


/*!
 * @brief map of motor data for each motor in the system
 */
    std::map<int, motor_data> motor_map_;
    std::mutex motor_data_mutex_;
    void set_target_rpms(const std::map<int, int>& target_rpms);


 /*!
  * @brief Radians per second to RPM conversion factor 1/0.10472
  */
    static constexpr double rads_sec_to_RPM{9.549};

/*!
 * @brief Max hardware RPM limit
 */
    static constexpr int MAX_RPM_LIMIT{1500};

/*!
 * @brief Hardware Interface
 */
 std::unique_ptr<vesc::Vesc> vesc_interface_;

};

}// namespace openrover::base_control
#endif // DIFF_DRIVE_CONTROLLER_LIFECYCLE_HPP
