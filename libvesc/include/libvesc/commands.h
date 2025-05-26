/*
    Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se

    This file is part of VESC Tool.

    VESC Tool is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    VESC Tool is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef COMMANDS_H
#define COMMANDS_H


#include "libvesc/packet.h"

namespace vesc {

    /**
     * @brief Handles the creation and processing of VESC commands.
     * 
     * This class is responsible for generating command packets to be sent to VESC motor controllers
     * and processing the received data packets. It supports various commands like getting firmware version,
     * setting motor parameters (duty cycle, current, RPM), and retrieving motor data.
     */
    class Commands {
    public:
        /**
         * @brief Default constructor for the Commands class.
         */
        Commands();

        /**
         * @brief Sets the limited mode for VESC communication.
         * @param is_limited True to enable limited mode, false otherwise.
         */
        inline void setLimitedMode(bool is_limited)
        {
            mIsLimitedMode = is_limited;
        }

        /**
         * @brief Checks if limited mode is enabled.
         * @return True if limited mode is enabled, false otherwise.
         */
        inline bool isLimitedMode()
        {
            return mIsLimitedMode;
        }

        /**
         * @brief Configures CAN bus forwarding.
         * @param sendCan True to enable forwarding messages over CAN bus, false otherwise.
         * @param id The CAN ID to use for forwarding. Defaults to -1 (no specific ID).
         * @return True if the configuration was successful, false otherwise.
         */
        bool setSendCan(bool sendCan, int32_t id = -1);

        /**
         * @brief Checks if CAN bus forwarding is enabled.
         * @return True if CAN forwarding is enabled, false otherwise.
         */
        inline bool getSendCan(){
            return mSendCan;
        }

        /**
         * @brief Sets the CAN ID for forwarding messages.
         * @param id The CAN ID to set.
         */
        inline void setCanSendId(int32_t id){
            mCanId = id;
        }

        /**
         * @brief Gets the current CAN ID used for forwarding messages.
         * @return The current CAN ID.
         */
        int32_t getCanSendId(){
            return mCanId;
        }

        /**
         * @brief Processes a received data packet.
         * @param message A vector of uint8_t representing the received message.
         */
        void processPacket(vector<uint8_t> &message);

        /**
         * @brief Requests the firmware version from the VESC.
         * @param vescPort The serial port object for communication with the VESC.
         */
        void getFwVersion(SerialPort vescPort);

        /**
         * @brief Requests general motor values from the VESC.
         * @param vescPort The serial port object for communication with the VESC.
         */
        void getValues(SerialPort vescPort);

        /**
         * @brief Sets the duty cycle of the motor.
         * @param dutyCycle The desired duty cycle (typically -1.0 to 1.0).
         * @param vescPort The serial port object for communication with the VESC.
         */
        void setDutyCycle(double dutyCycle, SerialPort vescPort);

        /**
         * @brief Sets the motor current.
         * @param current The desired motor current in Amperes.
         * @param vescPort The serial port object for communication with the VESC.
         */
        void setCurrent(double current, SerialPort vescPort);

        /**
         * @brief Sets the motor braking current.
         * @param current The desired braking current in Amperes.
         * @param vescPort The serial port object for communication with the VESC.
         */
        void setCurrentBrake(double current, SerialPort vescPort);

        /**
         * @brief Sets the motor RPM.
         * @param rpm The desired motor RPM.
         * @param vescPort The serial port object for communication with the VESC.
         */
        void setRpm(int32_t rpm, SerialPort vescPort);

        /**
         * @brief Sets the motor position.
         * @param pos The desired motor position (e.g., degrees or radians, depending on VESC configuration).
         * @param vescPort The serial port object for communication with the VESC.
         */
        void setPos(double pos, SerialPort vescPort);

        /**
         * @brief Sets the handbrake current.
         * @param current The desired handbrake current in Amperes.
         * @param vescPort The serial port object for communication with the VESC.
         */
        void setHandbrake(double current, SerialPort vescPort);

        /**
         * @brief Reboots the VESC.
         * @param vescPort The serial port object for communication with the VESC.
         */
        void reboot(SerialPort vescPort);

        /**
         * @brief Sends a keep-alive signal to the VESC.
         * @param vescPort The serial port object for communication with the VESC.
         */
        void sendAlive(SerialPort vescPort);

        /**
         * @brief Requests selective motor values from the VESC using a bitmask.
         * @param mask A bitmask specifying which values to retrieve.
         * @param vescPort The serial port object for communication with the VESC.
         */
        void getValuesSelective(uint32_t mask, SerialPort vescPort);

        /**
         * @brief Requests IMU data from the VESC using a bitmask.
         * @param mask A bitmask specifying which IMU data to retrieve.
         * @param vescPort The serial port object for communication with the VESC.
         */
        void getImuData(uint32_t mask, SerialPort vescPort);

        /**
         * @brief Gets a packet to request VESC ID and tachometer absolute value.
         * @return A Packet object configured for the request.
         */
        static Packet getVescIDpacket() { return Packet(COMM_GET_VALUES_SELECTIVE, castu32(MC_TACH_ABS | MC_VESC_ID)); }

        /**
         * @brief Gets a keep-alive packet.
         * @return A Packet object configured as a keep-alive signal.
         */
        static Packet getKeepAlivepacket() { return Packet(COMM_ALIVE); }

        /**
         * @brief Gets a packet to request selected motor data.
         * Includes RPM, motor current, motor temperature, input current, tachometer absolute value, and MOSFET temperature.
         * @return A Packet object configured for the request.
         */
        static Packet getSelectMotorpacket() {
            return Packet(COMM_GET_VALUES_SELECTIVE,
                          castu32(MC_RPM | MC_CURR_MOTOR | MC_TEMP_MOTOR | MC_CURR_IN |
                                  MC_TACH_ABS | MC_TEMP_MOS));
        }

        /**
         * @brief Gets a packet to request motor RPM and tachometer absolute value.
         * @return A Packet object configured for the request.
         */
        static Packet getMotorRPMpacket() { return Packet(COMM_GET_VALUES_SELECTIVE, castu32(MC_RPM | MC_TACH_ABS)); }

        /**
         * @brief Gets a packet to request all general motor values.
         * @return A Packet object configured for the request.
         */
        static Packet getMotorpacket() { return Packet(COMM_GET_VALUES); }

        /**
         * @brief Retrieves the stored motor controller data.
         * @return A reference to the MC_VALUES struct containing the latest motor data.
         */
        MC_VALUES &getMotorControllerData();

    private:
        /**
         * @brief Converts a fault code enum to its string representation.
         * @param fault The fault code to convert.
         * @return A string describing the fault.
         */
        string faultToStr(mc_fault_code fault);

        /**
         * @brief Flag indicating if messages should be forwarded over CAN bus.
         */
        bool mSendCan;
        /**
         * @brief CAN ID used for forwarding messages.
         */
        int32_t mCanId;
        /**
         * @brief Flag indicating if VESC communication is in limited mode.
         */
        bool mIsLimitedMode;
        /**
         * @brief Flag indicating if limited mode supports forwarding to all CAN devices.
         */
        bool mLimitedSupportsFwdAllCan;

        /**
         * @brief Timeout counter for general communication.
         */
        int32_t mTimeoutCount;
        /**
         * @brief Timeout counter for firmware version requests.
         */
        int32_t mTimeoutFwVer;
        /**
         * @brief Timeout counter for motor configuration requests.
         */
        int32_t mTimeoutMcconf;
        /**
         * @brief Timeout counter for application configuration requests.
         */
        int32_t mTimeoutAppconf;
        /**
         * @brief Timeout counter for value requests.
         */
        int32_t mTimeoutValues;
        /**
         * @brief Timeout counter for setup value requests.
         */
        int32_t mTimeoutValuesSetup;
        /**
         * @brief Timeout counter for IMU data requests.
         */
        int32_t mTimeoutImuData;
        /**
         * @brief Timeout counter for decoded PPM signal requests.
         */
        int32_t mTimeoutDecPpm;
        /**
         * @brief Timeout counter for decoded ADC signal requests.
         */
        int32_t mTimeoutDecAdc;
        /**
         * @brief Timeout counter for decoded Chuk (nunchuk) signal requests.
         */
        int32_t mTimeoutDecChuk;
        /**
         * @brief Timeout counter for CAN ping requests.
         */
        int32_t mTimeoutPingCan;

        /**
         * @brief Stores the latest motor controller data received from the VESC.
         */
        MC_VALUES motorControllerData;

    };

}


#endif // COMMANDS_H
