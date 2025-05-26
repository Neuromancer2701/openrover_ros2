
#ifndef VESC_H
#define VESC_H

#include <string>
#include <unordered_map>
#include "libvesc/commands.h"

namespace {
    using std::unordered_map;
    using std::string;
}

namespace vesc {
    /**
     * @brief Manages communication and control of VESC motor controllers.
     * 
     * This class provides an interface to find, configure, and control VESC-based motor controllers.
     * It supports operations like setting wheel RPM, duty cycle, and retrieving motor data.
     */
    class Vesc {
    public:
        /**
         * @brief Default constructor for the Vesc class.
         */
        Vesc();

        /**
         * @brief Default virtual destructor.
         */
        virtual ~Vesc() = default;

        /**
         * @brief Scans for and maps connected VESC motor controllers.
         */
        void FindandMapMotorControllers();

        /**
         * @brief Sets the desired RPM for specified wheels.
         * @param wheel_rpms An unordered_map where keys are wheel IDs and values are target RPMs.
         */
        void SetWheelsRPM(unordered_map<int, int> wheel_rpms);

        /**
         * @brief Sets the duty cycle for specified wheels.
         * @param wheel_duty An unordered_map where keys are wheel IDs and values are target duty cycles (0.0 to 1.0).
         */
        void SetWheelsDuty(unordered_map<int, double> wheel_duty);

        /**
         * @brief Retrieves the current RPM for all connected wheels.
         * @return An unordered_map where keys are wheel IDs and values are current RPMs.
         */
        unordered_map<int, double> GetWheelsRPM();

        /**
         * @brief Retrieves selected motor data (e.g., temperature, voltage) for specified motors.
         * @return An unordered_map where keys are wheel IDs and values are MC_VALUES structs containing motor data.
         */
        unordered_map<int, MC_VALUES> GetSelectMotorData();

        /**
         * @brief Retrieves all available motor data for all connected motors.
         * @return An unordered_map where keys are wheel IDs and values are MC_VALUES structs containing motor data.
         */
        unordered_map<int, MC_VALUES> GetAllMotorData();

        /**
         * @brief Checks if the VESC configuration is for a two-wheel drive setup.
         * @return True if configured for two-wheel drive, false otherwise.
         */
        bool isTwoWheelDrive();

        /**
         * @brief Checks if the VESC configuration is for a four-wheel drive setup.
         * @return True if configured for four-wheel drive, false otherwise.
         */
        bool isFourWheelDrive();

        /**
         * @brief Checks if any wheels are detected and mapped.
         * @return True if at least one wheel is found, false otherwise.
         */
        bool anyWheels() { return any_of(begin(wheel_found), end(wheel_found), [](auto kv) { return kv.second; }); }

        /**
         * @brief Returns the status of all potential wheel connections.
         * @return An unordered_map where keys are wheel IDs and values are booleans indicating if the wheel is found.
         */
        unordered_map<int, bool> Wheels() { return wheel_found; }

        /**
         * @brief Enum defining standard identifiers for wheels.
         */
        enum wheel_ids {
            left_back = 100,    ///< Identifier for the left back wheel.
            right_back = 200,   ///< Identifier for the right back wheel.
            left_front = 300,   ///< Identifier for the left front wheel.
            right_front = 400   ///< Identifier for the right front wheel.
        };


    private:
        /**
         * @brief Packet for VESC ID request.
         */
        const Packet m_VescIDPacket;
        /**
         * @brief Packet for sending keep-alive signals.
         */
        const Packet m_KeepAlivePacket;
        /**
         * @brief Packet for requesting all motor data.
         */
        const Packet m_AllMotorDataPacket;
        /**
         * @brief Packet for requesting selected motor data.
         */
        const Packet m_SelectmotorDataPacket;
        /**
         * @brief Packet for RPM related commands/data.
         */
        const Packet m_RPMPacket;

        /**
         * @brief Command handler object for VESC communication.
         */
        Commands cmd;

        /**
         * @brief Sends a packet and waits for a response over a specified serial port (string).
         * @param packet The packet to send.
         * @param port The serial port identifier (e.g., "/dev/ttyACM0").
         * @return True if the send and receive operation was successful, false otherwise.
         */
        bool SendAndReceive(const Packet &packet, const string &port);
        /**
         * @brief Sends a packet and waits for a response over a specified serial port (integer mapping).
         * @param packet The packet to send.
         * @param port The integer identifier for the port.
         * @return True if the send and receive operation was successful, false otherwise.
         */
        bool SendAndReceive(const Packet &packet, const int &port);

        /**
         * @brief Maps wheel IDs to their corresponding serial port strings.
         */
        unordered_map<int, string> wheel_ports;
        /**
         * @brief Maps wheel IDs to a boolean indicating if the wheel has been found/connected.
         */
        unordered_map<int, bool> wheel_found;
    };
}


#endif //VESC_H
