#ifndef DIFF_DRIVE_CONTROLLER_NODE_HPP
#define DIFF_DRIVE_CONTROLLER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include "libvesc/Vesc.h" // Assuming this provides vesc::Vesc

#include <memory>
#include <map>
#include <string>
#include <mutex>

namespace openrover::base_control
{

/**
 * @brief Alias for the return type of lifecycle node callbacks.
 * Simplifies the declaration of lifecycle callback methods, making the code cleaner.
 */
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief A ROS2 lifecycle node that implements differential drive kinematics for a robot.
 * 
 * This node subscribes to `cmd_vel` (geometry_msgs::msg::Twist) messages to receive velocity commands,
 * calculates the required wheel RPMs based on robot parameters (wheel separation and radius),
 * and communicates with VESC motor controllers via the `libvesc` library to set these RPMs. 
 * It also reads motor data (like RPM, voltage, temperature, faults) from the VESCs and can publish 
 * wheel velocities. The node follows the ROS2 lifecycle management for controlled startup, shutdown, 
 * activation, deactivation, and error handling.
 */
class DiffDriveController : public rclcpp_lifecycle::LifecycleNode {
public:
    /**
     * @brief Constructor for the DiffDriveController lifecycle node.
     * @param node_options Options for configuring the rclcpp_lifecycle::LifecycleNode, 
     *                     typically passed from the component container or node instantiation process.
     */
    DiffDriveController(const rclcpp::NodeOptions & node_options);

private:
    /**
     * @brief Callback function for the `cmd_vel` subscriber.
     * 
     * This method is invoked when a new geometry_msgs::msg::Twist message is received on the "cmd_vel" topic.
     * It calculates the target RPM for the left and right wheels based on the incoming
     * linear velocity (msg->linear.x) and angular velocity (msg->angular.z) using differential drive kinematics.
     * The calculated RPMs are then stored internally to be written to the hardware by `write_hardware()`.
     * @param msg A shared pointer to the received geometry_msgs::msg::Twist message.
     */
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     * @brief Helper function to publish current wheel velocities.
     * 
     * This function reads the actual RPM values from the `motor_map_` (which is updated by `read_hardware()`)
     * and publishes them as std_msgs::msg::Float64 messages on the `left_wheel_pub_` and `right_wheel_pub_` topics.
     * The published value represents the current estimated speed of each wheel.
     */
    void publishWheelVelocity();

    // Lifecycle management callbacks
    /**
     * @brief Lifecycle callback executed when the node transitions to the "configuring" state.
     * 
     * This method is responsible for:
     * - Initializing parameters from the ROS2 parameters server (e.g., `wheel_separation_`, `wheel_radius_`).
     * - Setting up ROS2 publishers (`left_wheel_pub_`, `right_wheel_pub_`).
     * - Setting up ROS2 subscribers (`cmd_vel_sub_`).
     * - Initializing the VESC hardware interface (`vesc_interface_`).
     * - Creating the timer (`timer_`) for periodic hardware communication.
     * @param previous_state The previous lifecycle state of the node.
     * @return CallbackReturn::SUCCESS if configuration is successful.
     * @return CallbackReturn::ERROR if an unrecoverable error occurs during configuration.
     * @return CallbackReturn::FAILURE if configuration fails due to invalid parameters or other issues.
     */
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state);

    /**
     * @brief Lifecycle callback executed when the node transitions to the "activating" state.
     * 
     * This method activates the node's main functionality by:
     * - Activating the lifecycle publishers, allowing them to publish messages.
     * - Starting the VESC interface communication (e.g., finding and mapping motor controllers if not done in on_configure).
     * - Starting the periodic timer (`timer_`) which drives the read/write hardware loop.
     * @param previous_state The previous lifecycle state of the node.
     * @return CallbackReturn::SUCCESS if activation is successful.
     * @return CallbackReturn::ERROR if an unrecoverable error occurs during activation.
     * @return CallbackReturn::FAILURE if activation fails (e.g., hardware not responding).
     */
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state);

    /**
     * @brief Lifecycle callback executed when the node transitions to the "deactivating" state.
     * 
     * This method deactivates the node's main functionality by:
     * - Deactivating the lifecycle publishers, preventing them from publishing further messages.
     * - Stopping the VESC interface communication (e.g., setting motor RPMs to zero for safety).
     * - Stopping the periodic timer (`timer_`).
     * @param previous_state The previous lifecycle state of the node.
     * @return CallbackReturn::SUCCESS if deactivation is successful.
     * @return CallbackReturn::ERROR if an unrecoverable error occurs during deactivation.
     * @return CallbackReturn::FAILURE if deactivation fails.
     */
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state);

    /**
     * @brief Lifecycle callback executed when the node transitions to the "cleaningup" state.
     * 
     * This method releases resources that were acquired during the `on_configure` phase. For example:
     * - Resetting or releasing publishers, subscribers, and services.
     * - Shutting down and releasing the VESC interface (`vesc_interface_`).
     * - Cancelling and releasing the timer (`timer_`).
     * @param previous_state The previous lifecycle state of the node.
     * @return CallbackReturn::SUCCESS if cleanup is successful.
     * @return CallbackReturn::ERROR if an unrecoverable error occurs during cleanup.
     * @return CallbackReturn::FAILURE if cleanup fails.
     */
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state);

    /**
     * @brief Lifecycle callback executed when the node transitions to the "shuttingdown" state.
     * 
     * This method performs any final cleanup required before the node is destroyed.
     * It should ensure all resources are properly released, similar to `on_cleanup`.
     * @param previous_state The previous lifecycle state of the node.
     * @return CallbackReturn::SUCCESS if shutdown is successful.
     * @return CallbackReturn::ERROR if an unrecoverable error occurs during shutdown.
     * @return CallbackReturn::FAILURE if shutdown fails.
     */
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state);

    /**
     * @brief Lifecycle callback executed when the node encounters an error and transitions to the "errorprocessing" state.
     * 
     * This method is intended to handle errors that may have occurred during the node's operation,
     * potentially attempting recovery or ensuring a safe state.
     * @param previous_state The previous lifecycle state of the node.
     * @return CallbackReturn::SUCCESS if error processing is handled successfully.
     * @return CallbackReturn::ERROR if an unrecoverable error occurs during error processing.
     * @return CallbackReturn::FAILURE if error processing fails.
     */
    CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state);

    /**
     * @brief Timer callback function executed periodically at a fixed rate.
     * 
     * This method orchestrates the continuous interaction with the hardware. It calls:
     * - `read_hardware()` to get the latest telemetry from VESC motor controllers.
     * - `write_hardware()` to send updated RPM commands to the VESC motor controllers.
     * - `publishWheelVelocity()` to publish the current wheel speeds.
     */
    void timerCallback();

    /**
     * @brief Reads data from the VESC motor controllers.
     * 
     * This method communicates with the `vesc_interface_` to retrieve various telemetry data
     * such as input voltage, motor temperature, motor current, absolute tachometer readings,
     * fault codes, and current RPM for each connected VESC. The retrieved data is then stored
     * in the `motor_map_` member variable, protected by `motor_data_mutex_` for thread safety.
     */
   void read_hardware();

   /**
    * @brief Writes target RPM values to the VESC motor controllers.
    * 
    * This method retrieves the target RPMs stored in the `motor_map_` (which are typically updated by
    * `cmdVelCallback` or `set_target_rpms`) and sends these commands to the respective
    * VESC motor controllers via the `vesc_interface_`. RPM values may be capped by `MAX_RPM_LIMIT`.
    */
   void write_hardware();

    /**
     * @brief ROS2 subscriber for receiving velocity commands from the "cmd_vel" topic.
     * It listens for `geometry_msgs::msg::Twist` messages, which are processed by `cmdVelCallback`.
     */
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    /**
     * @brief ROS2 lifecycle publisher for the left wheel's velocity.
     * It publishes `std_msgs::msg::Float64` messages, typically representing RPM or rad/s,
     * on a topic like "left_wheel_velocity".
     */
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr left_wheel_pub_;
    /**
     * @brief ROS2 lifecycle publisher for the right wheel's velocity.
     * It publishes `std_msgs::msg::Float64` messages, typically representing RPM or rad/s,
     * on a topic like "right_wheel_velocity".
     */
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr right_wheel_pub_;

    /**
     * @brief ROS2 timer for periodically executing the `timerCallback` method.
     * This timer drives the continuous read/write cycle with the VESC motor controllers and publishing of data.
     */
    rclcpp::TimerBase::SharedPtr timer_;

    /**
     * @brief The separation distance between the centers of the robot's driving wheels, measured in meters.
     * This parameter is crucial for accurate differential drive kinematics calculations. It is typically loaded
     * from ROS2 parameters during the `on_configure` lifecycle state.
     */
    double wheel_separation_;
    /**
     * @brief The radius of the robot's wheels, measured in meters.
     * This parameter is also essential for differential drive kinematics calculations (converting linear/angular
     * velocity to wheel RPM). It is typically loaded from ROS2 parameters during `on_configure`.
     */
    double wheel_radius_;

    /**
     * @brief Structure to hold telemetry data read from a motor controller and its target RPM.
     */
    struct motor_data
    {
        double volt_in;         ///< Input voltage to the VESC (Volts).
        double temp_motor;      ///< Motor temperature (°C).
        double current_motor;   ///< Motor current (Amperes).
        int tachometer_abs;     ///< Absolute tachometer reading from the VESC (counts).
        int fault_code;         ///< VESC fault code (integer representation).
        std::string fault_str;  ///< String description of the VESC fault code.
        double rpm;             ///< Current RPM of the motor, read from VESC.
        int target_rpm;         ///< Target RPM to be set for the motor, calculated from cmd_vel.
    };

    /**
     * @brief Map storing `motor_data` for each motor controller, keyed by its VESC ID.
     * This map provides a centralized place to store both telemetry from and commands to the motors.
     * Access to this map is protected by `motor_data_mutex_` to ensure thread safety.
     */
    std::map<int, motor_data> motor_map_;
    /**
     * @brief Mutex to protect concurrent access to `motor_map_`, ensuring thread safety
     * when reading from or writing to the motor data.
     */
    std::mutex motor_data_mutex_;
    /**
     * @brief Sets the target RPMs for multiple motors in a thread-safe manner.
     * 
     * This method updates the `target_rpm` field within the `motor_map_` for each
     * VESC ID specified in the input map. It acquires a lock on `motor_data_mutex_` before modifying the map.
     * @param target_rpms A constant reference to a map where keys are VESC IDs (int) 
     *                    and values are the target RPMs (int) for those VESCs.
     */
    void set_target_rpms(const std::map<int, int>& target_rpms);

    /**
     * @brief Conversion factor from radians per second (rad/s) to revolutions per minute (RPM).
     * This constant is used when converting calculated wheel angular velocities to RPM values
     * suitable for the VESC motor controllers. Value is approximately 9.549 (calculated as 60 / (2 * PI)).
     */
    static constexpr double rads_sec_to_RPM{9.549};

    /**
     * @brief Maximum hardware RPM limit.
     * This constant is used to cap the target RPM values sent to the VESC motor controllers,
     * preventing them from exceeding safe operational limits or configured maximums.
     */
    static constexpr int MAX_RPM_LIMIT{1500};

    /**
     * @brief Unique pointer to the VESC hardware interface object (from the `libvesc` library).
     * This interface is initialized during `on_configure` and used for all direct communication 
     * (sending commands, receiving data) with the VESC motor controllers.
     */
    std::unique_ptr<vesc::Vesc> vesc_interface_;

}; // class DiffDriveController

} // namespace openrover::base_control
#endif // DIFF_DRIVE_CONTROLLER_LIFECYCLE_HPP
