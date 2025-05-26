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

#include "libvesc/commands.h"

using namespace vesc;

Commands::Commands()
        : mSendCan(false), mCanId(-1), mIsLimitedMode(false), mTimeoutCount(100), mTimeoutFwVer(0), mTimeoutMcconf(0),
          mTimeoutAppconf(0), mTimeoutValues(0), mTimeoutValuesSetup(0), mTimeoutImuData(0), mTimeoutDecPpm(0),
          mTimeoutDecAdc(0), mTimeoutDecChuk(0), mTimeoutPingCan(0) {


}

bool Commands::setSendCan(bool sendCan, int32_t id) { // Explicitly use int32_t for clarity if it matches header
    if (id >= 0) {
        mCanId = id;
        mSendCan = sendCan;
        return true;
    }
    // No else needed as it returns false if condition is not met.
    return false;
}

void Commands::processPacket(vector<uint8_t> &message) {
    // It's important that Packet::pop handles empty 'message' gracefully or it's checked before.
    if (message.empty()) {
        // Or handle error appropriately
        return; 
    }

    uint8_t id = 0; // Use fixed-size integer type for clarity
    Packet::pop(message, id);

    switch (static_cast<COMM_PACKET_ID>(id)) { // Cast id to enum type for switch
        case COMM_FW_VERSION: {
            mTimeoutFwVer = 0;
            // Using auto for type deduction where types are clear from context or initialization
            auto fw_major = int8_t{-1}; // Assuming char might be signed/unsigned based on platform.
            auto fw_minor = int8_t{-1}; // Using int8_t for explicit size and signedness.
            // string hw; // hw is not used, commented out.
            // vector<char> uuid; // uuid is not used, commented out.
            // bool isPaired = false; // isPaired is not used, commented out.

            if (message.size() >= 2) {
                Packet::pop(message, fw_major);
                Packet::pop(message, fw_minor);
                // hw = message.vbPopFrontString(); // This method and similar ones are commented out in original
            }

            // The following blocks for uuid and isPaired are commented out in the original code.
            // If they were active, similar modernization would apply.
            // if (message.size() >= 12) {
            //     //uuid = message.left(12);
            //     //message.erase(12);
            // }
            // if (message.size() >= 1) {
            //     //isPaired = message.vbPopFrontInt8();
            // }
        }
            break; // Ensure break is present for each case

        case COMM_GET_VALUES:
        case COMM_GET_VALUES_SELECTIVE: {
            mTimeoutValues = 0;

            uint32_t mask = 0xFFFFFFFF; // Use fixed-size integer type
            if (static_cast<COMM_PACKET_ID>(id) == COMM_GET_VALUES_SELECTIVE) {
                Packet::pop(message, mask);
            }

            // Using if statements with consistent mask checking style
            if ((mask & MC_TEMP_MOS) == MC_TEMP_MOS) { // Enum values are already powers of 2
                motorControllerData.temp_mos = Packet::popDouble16(message, 1e1);
            }
            if ((mask & MC_TEMP_MOTOR) == MC_TEMP_MOTOR) {
                motorControllerData.temp_motor = Packet::popDouble16(message, 1e1);
            }
            if ((mask & MC_CURR_MOTOR) == MC_CURR_MOTOR) {
                motorControllerData.current_motor = Packet::popDouble32(message, 1e2);
            }
            if ((mask & MC_CURR_IN) == MC_CURR_IN) {
                motorControllerData.current_in = Packet::popDouble32(message, 1e2);
            }
            if ((mask & MC_ID) == MC_ID) {
                motorControllerData.id = Packet::popDouble32(message, 1e2);
            }
            if ((mask & MC_IQ) == MC_IQ) {
                motorControllerData.iq = Packet::popDouble32(message, 1e2);
            }
            if ((mask & MC_DUTY_NOW) == MC_DUTY_NOW) {
                motorControllerData.duty_now = Packet::popDouble16(message, 1e3);
            }
            if ((mask & MC_RPM) == MC_RPM) {
                // Assuming popDouble32 returns a type convertible to double (motorControllerData.rpm type)
                motorControllerData.rpm = Packet::popDouble32(message, 1e0);
            }
            if ((mask & MC_V_IN) == MC_V_IN) {
                motorControllerData.v_in = Packet::popDouble16(message, 1e1);
            }
            if ((mask & MC_AMP_HRS) == MC_AMP_HRS) {
                motorControllerData.amp_hours = Packet::popDouble32(message, 1e4);
            }
            if ((mask & MC_AMP_HRS_CH) == MC_AMP_HRS_CH) {
                motorControllerData.amp_hours_charged = Packet::popDouble32(message, 1e4);
            }
            if ((mask & MC_WATT_HRS) == MC_WATT_HRS) {
                motorControllerData.watt_hours = Packet::popDouble32(message, 1e4);
            }
            if ((mask & MC_WATT_HRS_CH) == MC_WATT_HRS_CH) {
                motorControllerData.watt_hours_charged = Packet::popDouble32(message, 1e4);
            }
            if ((mask & MC_TACH) == MC_TACH) {
                Packet::pop(message, motorControllerData.tachometer); // Assuming Packet::pop deduces type correctly
            }
            if ((mask & MC_TACH_ABS) == MC_TACH_ABS) {
                Packet::pop(message, motorControllerData.tachometer_abs);
            }
            if ((mask & MC_FAULT_CODE) == MC_FAULT_CODE) {
                uint8_t temp_fault_code = 0; // Use fixed-size type
                Packet::pop(message, temp_fault_code);
                motorControllerData.fault_code = static_cast<mc_fault_code>(temp_fault_code);
                motorControllerData.fault_str = faultToStr(motorControllerData.fault_code);
            }

            // Check message size before attempting to pop more data
            if (message.size() >= 4 && (mask & MC_POSITION) == MC_POSITION) {
                 motorControllerData.position = Packet::popDouble32(message, 1e6);
            } else if ((mask & MC_POSITION) == MC_POSITION) { // If mask requests it but not enough data
                motorControllerData.position = -1.0; // Default or error value
            }


            if (message.size() >= 1 && (mask & MC_VESC_ID) == MC_VESC_ID) {
                uint8_t temp_vesc_id = 0; // Use fixed-size type
                Packet::pop(message, temp_vesc_id);
                motorControllerData.vesc_id = static_cast<int>(temp_vesc_id); // vesc_id is int in struct
            } else if ((mask & MC_VESC_ID) == MC_VESC_ID) { // If mask requests it but not enough data
                motorControllerData.vesc_id = 255; // Default or error value
            }


            if (message.size() >= 6 && (mask & MC_TEMP_MOS_123) == MC_TEMP_MOS_123) {
                motorControllerData.temp_mos_1 = Packet::popDouble16(message, 1e1);
                motorControllerData.temp_mos_2 = Packet::popDouble16(message, 1e1);
                motorControllerData.temp_mos_3 = Packet::popDouble16(message, 1e1);
            }
        }
            break; // Ensure break is present for each case
#if 0
            case COMM_GET_IMU_DATA:
                {
                mTimeoutImuData = 0;

                IMU_VALUES values;

                uint32_t mask = message.vbPopFrontUint16();

                if (mask & ((uint32_t)1 << 0)) {
                    values.roll = message.vbPopFrontDouble32Auto();
                }
                if (mask & ((uint32_t)1 << 1)) {
                    values.pitch = message.vbPopFrontDouble32Auto();
                }
                if (mask & ((uint32_t)1 << 2)) {
                    values.yaw = message.vbPopFrontDouble32Auto();
                }

                if (mask & ((uint32_t)1 << 3)) {
                    values.accX = message.vbPopFrontDouble32Auto();
                }
                if (mask & ((uint32_t)1 << 4)) {
                    values.accY = message.vbPopFrontDouble32Auto();
                }
                if (mask & ((uint32_t)1 << 5)) {
                    values.accZ = message.vbPopFrontDouble32Auto();
                }

                if (mask & ((uint32_t)1 << 6)) {
                    values.gyroX = message.vbPopFrontDouble32Auto();
                }
                if (mask & ((uint32_t)1 << 7)) {
                    values.gyroY = message.vbPopFrontDouble32Auto();
                }
                if (mask & ((uint32_t)1 << 8)) {
                    values.gyroZ = message.vbPopFrontDouble32Auto();
                }

                if (mask & ((uint32_t)1 << 9)) {
                    values.magX = message.vbPopFrontDouble32Auto();
                }
                if (mask & ((uint32_t)1 << 10)) {
                    values.magY = message.vbPopFrontDouble32Auto();
                }
                if (mask & ((uint32_t)1 << 11)) {
                    values.magZ = message.vbPopFrontDouble32Auto();
                }

                if (mask & ((uint32_t)1 << 12)) {
                    values.q0 = message.vbPopFrontDouble32Auto();
                }
                if (mask & ((uint32_t)1 << 13)) {
                    values.q1 = message.vbPopFrontDouble32Auto();
                }
                if (mask & ((uint32_t)1 << 14)) {
                    values.q2 = message.vbPopFrontDouble32Auto();
                }
                if (mask & ((uint32_t)1 << 15)) {
                    values.q3 = message.vbPopFrontDouble32Auto();
                }

            } break;
#endif
        default:
            break;
    }
}

void Commands::getFwVersion(SerialPort vescPort) {
    if (mTimeoutFwVer > 0) {
        return;
    }

    mTimeoutFwVer = mTimeoutCount;

    vescPort.Write(Packet(COMM_FW_VERSION).createPacket());
}

void Commands::getValues(SerialPort vescPort) {
    vescPort.Write(Packet(COMM_GET_VALUES).createPacket());
}

void Commands::setDutyCycle(double dutyCycle, SerialPort vescPort) {
    vescPort.Write(Packet(COMM_SET_DUTY, dutyCycle, 1e5).createPacket());
}

void Commands::setCurrent(double current, SerialPort vescPort) {
    vescPort.Write(Packet(COMM_SET_CURRENT, current, 1e3).createPacket());
}

void Commands::setCurrentBrake(double current, SerialPort vescPort) {
    vescPort.Write(Packet(COMM_SET_CURRENT_BRAKE, current, 1e3).createPacket());
}

void Commands::setRpm(int32_t rpm, SerialPort vescPort) { // Explicitly use int32_t if it matches header
    // The cast to unsigned might be platform-dependent or specific to Packet constructor.
    // Ensuring rpm is not negative before casting to unsigned is safer if Packet expects non-negative.
    // However, if VESC protocol uses negative RPM for direction, this cast is part of the protocol.
    vescPort.Write(Packet(COMM_SET_RPM, static_cast<uint32_t>(rpm)).createPacket());
}

void Commands::setPos(double pos, SerialPort vescPort) {
    // Ensure scale factor 1e6 is appropriate for the precision required by VESC for position.
    vescPort.Write(Packet(COMM_SET_POS, pos, 1e6).createPacket());
}

void Commands::setHandbrake(double current, SerialPort vescPort) {
    vescPort.Write(Packet(COMM_SET_HANDBRAKE, current, 1e3).createPacket());
}

void Commands::reboot(SerialPort vescPort) {
    vescPort.Write(Packet(COMM_REBOOT).createPacket());
}

void Commands::sendAlive(SerialPort vescPort) {
    vescPort.Write(Packet(COMM_ALIVE).createPacket());
}

void Commands::getValuesSelective(uint32_t mask, SerialPort vescPort) { // Use fixed-size type
    vescPort.Write(Packet(COMM_GET_VALUES_SELECTIVE, mask).createPacket());
}

void Commands::getImuData(uint32_t mask, SerialPort vescPort) { // Use fixed-size type
    if (mTimeoutImuData > 0) {
        return; // Already waiting for a response or timeout active
    }

    mTimeoutImuData = mTimeoutCount; // Reset timeout

    // The cast to unsigned short (uint16_t) might truncate the mask if it's larger.
    // This should match what the VESC firmware expects for this command.
    // If VESC expects uint16_t, then this is correct.
    vescPort.Write(Packet(COMM_GET_IMU_DATA, static_cast<uint16_t>(mask)).createPacket());
}

// This function returns std::string, so std::string_view is not directly applicable for return type
// without changing API. Parameter is an enum, so no change there.
// Using std::string_view for the return and having callers convert to std::string
// could be a C++17+ optimization if this function is hot and strings are short-lived.
// However, for simplicity and to avoid breaking existing uses that expect std::string, keep as is.
std::string Commands::faultToStr(mc_fault_code fault) {
    // Using a switch statement here is clear and efficient for enums.
    // No fallthroughs are intended.
    switch (fault) {
        case FAULT_CODE_NONE:
            return "FAULT_CODE_NONE"; // Potentially make these string_literals (C++14/17) or string_view (C++17)
                                      // if performance is critical and to avoid std::string allocations.
                                      // For now, returning string literals is fine, they convert to std::string.
        case FAULT_CODE_OVER_VOLTAGE:
            return "FAULT_CODE_OVER_VOLTAGE";
        case FAULT_CODE_UNDER_VOLTAGE:
            return "FAULT_CODE_UNDER_VOLTAGE";
        case FAULT_CODE_DRV:
            return "FAULT_CODE_DRV";
        case FAULT_CODE_ABS_OVER_CURRENT:
            return "FAULT_CODE_ABS_OVER_CURRENT";
        case FAULT_CODE_OVER_TEMP_FET:
            return "FAULT_CODE_OVER_TEMP_FET";
        case FAULT_CODE_OVER_TEMP_MOTOR:
            return "FAULT_CODE_OVER_TEMP_MOTOR";
        case FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE:
            return "FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE";
        case FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE:
            return "FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE";
        case FAULT_CODE_MCU_UNDER_VOLTAGE:
            return "FAULT_CODE_MCU_UNDER_VOLTAGE";
        case FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET:
            return "FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET";
        case FAULT_CODE_ENCODER_SPI:
            return "FAULT_CODE_ENCODER_SPI";
        case FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE:
            return "FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE";
        case FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE:
            return "FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE";
        case FAULT_CODE_FLASH_CORRUPTION:
            return "FAULT_CODE_FLASH_CORRUPTION";
        case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1:
            return "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1";
        case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2:
            return "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2";
        case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3:
            return "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3";
        case FAULT_CODE_UNBALANCED_CURRENTS:
            return "FAULT_CODE_UNBALANCED_CURRENTS";
        default:
            // Consider logging an unknown fault code for debugging purposes.
            return "Unknown fault"; // Or perhaps "UNKNOWN_FAULT_CODE_" + std::to_string(static_cast<int>(fault));
    }
}

// Returns a reference to a member variable. No significant modernization applicable here
// beyond ensuring MC_VALUES itself is modern if it were defined in this scope.
MC_VALUES &Commands::getMotorControllerData() {
    return motorControllerData;
}
