/*
    Copyright 2016 - 2018 Benjamin Vedder	benjamin@vedder.se

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

#ifndef DATATYPES_H
#define DATATYPES_H

#include <string>

namespace {
    using std::string;
}

/**
 * @brief Describes the type of VESC transmission data.
 * Used to specify the data type when sending or receiving data from VESC.
 */
enum VESC_TX_T 
{
    VESC_TX_UNDEFINED = 0,      ///< Undefined transmission type.
    VESC_TX_UINT8,              ///< Unsigned 8-bit integer.
    VESC_TX_INT8,               ///< Signed 8-bit integer.
    VESC_TX_UINT16,             ///< Unsigned 16-bit integer.
    VESC_TX_INT16,              ///< Signed 16-bit integer.
    VESC_TX_UINT32,             ///< Unsigned 32-bit integer.
    VESC_TX_INT32,              ///< Signed 32-bit integer.
    VESC_TX_DOUBLE16,           ///< 16-bit floating point number.
    VESC_TX_DOUBLE32,           ///< 32-bit floating point number.
    VESC_TX_DOUBLE32_AUTO       ///< Automatic 32-bit floating point number (precision may vary).
};

/**
 * @brief Enumerates motor controller fault codes.
 * These codes indicate specific error conditions reported by the VESC.
 */
enum mc_fault_code 
{
    FAULT_CODE_NONE = 0,                            ///< No fault.
    FAULT_CODE_OVER_VOLTAGE,                        ///< Over voltage fault.
    FAULT_CODE_UNDER_VOLTAGE,                       ///< Under voltage fault.
    FAULT_CODE_DRV,                                 ///< DRV (MOSFET driver) fault.
    FAULT_CODE_ABS_OVER_CURRENT,                    ///< Absolute over current fault.
    FAULT_CODE_OVER_TEMP_FET,                       ///< FET (MOSFET) over temperature fault.
    FAULT_CODE_OVER_TEMP_MOTOR,                     ///< Motor over temperature fault.
    FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE,            ///< Gate driver over voltage fault.
    FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE,           ///< Gate driver under voltage fault.
    FAULT_CODE_MCU_UNDER_VOLTAGE,                   ///< MCU under voltage fault.
    FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET,         ///< System booted from a watchdog reset.
    FAULT_CODE_ENCODER_SPI,                         ///< Encoder SPI communication fault.
    FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE,  ///< Encoder sin/cos signal amplitude below minimum.
    FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE,  ///< Encoder sin/cos signal amplitude above maximum.
    FAULT_CODE_FLASH_CORRUPTION,                    ///< Flash memory corruption detected.
    FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1,        ///< High offset detected in current sensor 1.
    FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2,        ///< High offset detected in current sensor 2.
    FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3,        ///< High offset detected in current sensor 3.
    FAULT_CODE_UNBALANCED_CURRENTS                  ///< Unbalanced motor phase currents detected.
};

/**
 * @brief Bitmask for selecting specific motor controller values to retrieve.
 * Used with commands like `COMM_GET_VALUES_SELECTIVE`.
 */
enum SelectMCValues
{
    MC_TEMP_MOS     = 0b0000000000000000001,   ///< MOSFET temperature. (1 << 0)
    MC_TEMP_MOTOR   = 0b0000000000000000010,   ///< Motor temperature. (1 << 1)
    MC_CURR_MOTOR   = 0b0000000000000000100,   ///< Motor current. (1 << 2)
    MC_CURR_IN      = 0b0000000000000001000,   ///< Input current. (1 << 3)
    MC_ID           = 0b0000000000000010000,   ///< D-axis current. (1 << 4)
    MC_IQ           = 0b0000000000000100000,   ///< Q-axis current. (1 << 5)
    MC_DUTY_NOW     = 0b0000000000001000000,   ///< Current duty cycle. (1 << 6)
    MC_RPM          = 0b0000000000010000000,   ///< Motor RPM. (1 << 7)
    MC_V_IN         = 0b0000000000100000000,   ///< Input voltage. (1 << 8)
    MC_AMP_HRS      = 0b0000000001000000000,   ///< Consumed Ampere-hours. (1 << 9)
    MC_AMP_HRS_CH   = 0b0000000010000000000,   ///< Charged Ampere-hours. (1 << 10)
    MC_WATT_HRS     = 0b0000000100000000000,   ///< Consumed Watt-hours. (1 << 11)
    MC_WATT_HRS_CH  = 0b0000001000000000000,   ///< Charged Watt-hours. (1 << 12)
    MC_TACH         = 0b0000010000000000000,   ///< Tachometer value (cumulative). (1 << 13)
    MC_TACH_ABS     = 0b0000100000000000000,   ///< Absolute tachometer value. (1 << 14)
    MC_FAULT_CODE   = 0b0001000000000000000,   ///< Current fault code. (1 << 15)
    MC_POSITION     = 0b0010000000000000000,   ///< Motor position. (1 << 16)
    MC_VESC_ID      = 0b0100000000000000000,   ///< VESC CAN ID. (1 << 17)
    MC_TEMP_MOS_123 = 0b1000000000000000000,   ///< Temperatures of individual MOSFETs. (1 << 18)
};

/**
 * @brief Structure to hold motor controller values.
 * 
 * This structure contains various telemetry and status data from the motor controller,
 * such as temperatures, currents, RPM, and fault codes.
 */
struct MC_VALUES
{
    /**
     * @brief Default constructor initializing all members to zero or default values.
     */
    MC_VALUES()
    {
        v_in = 0.0;
        temp_mos = 0.0;
        temp_mos_1 = 0.0;
        temp_mos_2 = 0.0;
        temp_mos_3 = 0.0;
        temp_motor = 0.0;
        current_motor = 0.0;
        current_in = 0.0;
        id = 0.0;
        iq = 0.0;
        rpm = 0.0;
        duty_now = 0.0;
        amp_hours = 0.0;
        amp_hours_charged = 0.0;
        watt_hours = 0.0;
        watt_hours_charged = 0.0;
        tachometer = 0;
        tachometer_abs = 0;
        position = 0.0;
        fault_code = FAULT_CODE_NONE;
        vesc_id = 0;
        fault_str = "";
    }

    double v_in;                ///< Input voltage to the VESC (Volts).
    double temp_mos;            ///< Average MOSFET temperature (°C).
    double temp_mos_1;          ///< Temperature of MOSFET 1 (°C).
    double temp_mos_2;          ///< Temperature of MOSFET 2 (°C).
    double temp_mos_3;          ///< Temperature of MOSFET 3 (°C).
    double temp_motor;          ///< Motor temperature (°C).
    double current_motor;       ///< Motor phase current (Amperes).
    double current_in;          ///< Input current to the VESC (Amperes).
    double id;                  ///< D-axis current (Amperes).
    double iq;                  ///< Q-axis current (Amperes).
    double rpm;                 ///< Motor speed (Revolutions Per Minute).
    double duty_now;            ///< Current duty cycle (0.0 to 1.0).
    double amp_hours;           ///< Consumed energy (Ampere-hours).
    double amp_hours_charged;   ///< Regenerated energy (Ampere-hours).
    double watt_hours;          ///< Consumed energy (Watt-hours).
    double watt_hours_charged;  ///< Regenerated energy (Watt-hours).
    int tachometer;             ///< Cumulative tachometer value (counts).
    int tachometer_abs;         ///< Absolute tachometer value (counts).
    double position;            ///< Motor position (e.g., degrees or radians).
    mc_fault_code fault_code;   ///< Current fault code.
    int vesc_id;                ///< VESC CAN ID.
    string fault_str;           ///< String representation of the fault code.
};

/**
 * @brief Structure to hold Inertial Measurement Unit (IMU) values.
 * 
 * This structure contains data related to orientation and motion, such as roll, pitch, yaw,
 * acceleration, gyroscope, magnetometer readings, and quaternion data.
 */
struct IMU_VALUES {
    /**
     * @brief Default constructor initializing all members to zero or default (quaternion identity).
     */
    IMU_VALUES()
    {
        roll = 0;
        pitch = 0;
        yaw = 0;
        accX = 0;
        accY = 0;
        accZ = 0;
        gyroX = 0;
        gyroY = 0;
        gyroZ = 0;
        magX = 0;
        magY = 0;
        magZ = 0;
        q0 = 1; // Identity quaternion w component
        q1 = 0; // Identity quaternion x component
        q2 = 0; // Identity quaternion y component
        q3 = 0; // Identity quaternion z component
    }

    double roll;    ///< Roll angle (degrees or radians).
    double pitch;   ///< Pitch angle (degrees or radians).
    double yaw;     ///< Yaw angle (degrees or radians).

    double accX;    ///< Acceleration along X-axis (m/s^2 or g).
    double accY;    ///< Acceleration along Y-axis (m/s^2 or g).
    double accZ;    ///< Acceleration along Z-axis (m/s^2 or g).

    double gyroX;   ///< Gyroscope reading along X-axis (deg/s or rad/s).
    double gyroY;   ///< Gyroscope reading along Y-axis (deg/s or rad/s).
    double gyroZ;   ///< Gyroscope reading along Z-axis (deg/s or rad/s).

    double magX;    ///< Magnetometer reading along X-axis (Gauss or Tesla).
    double magY;    ///< Magnetometer reading along Y-axis (Gauss or Tesla).
    double magZ;    ///< Magnetometer reading along Z-axis (Gauss or Tesla).

    double q0;      ///< Quaternion component w (scalar part).
    double q1;      ///< Quaternion component x (vector part).
    double q2;      ///< Quaternion component y (vector part).
    double q3;      ///< Quaternion component z (vector part).
};


/**
 * @brief Enumerates debug sampling modes.
 * These modes control how and when debug data is sampled and transmitted.
 */
enum debug_sampling_mode
{
    DEBUG_SAMPLING_OFF = 0,                 ///< Debug sampling is off.
    DEBUG_SAMPLING_NOW,                     ///< Sample debug data immediately.
    DEBUG_SAMPLING_START,                   ///< Start continuous debug sampling.
    DEBUG_SAMPLING_TRIGGER_START,           ///< Start sampling on a trigger event.
    DEBUG_SAMPLING_TRIGGER_FAULT,           ///< Start sampling on a fault trigger.
    DEBUG_SAMPLING_TRIGGER_START_NOSEND,    ///< Start sampling on trigger, do not send automatically.
    DEBUG_SAMPLING_TRIGGER_FAULT_NOSEND,    ///< Start sampling on fault trigger, do not send automatically.
    DEBUG_SAMPLING_SEND_LAST_SAMPLES        ///< Send the last collected samples.
};

/**
 * @brief Enumerates communication packet identifiers.
 * These IDs define the type of command or data being sent or received.
 */
enum COMM_PACKET_ID
{
    COMM_FW_VERSION = 0,                        ///< Request firmware version.
    COMM_JUMP_TO_BOOTLOADER,                    ///< Command VESC to jump to bootloader mode.
    COMM_ERASE_NEW_APP,                         ///< Command VESC to erase application memory.
    COMM_WRITE_NEW_APP_DATA,                    ///< Command to write new application data.
    COMM_GET_VALUES,                            ///< Request general VESC status values.
    COMM_SET_DUTY,                              ///< Set motor duty cycle.
    COMM_SET_CURRENT,                           ///< Set motor current.
    COMM_SET_CURRENT_BRAKE,                     ///< Set motor brake current.
    COMM_SET_RPM,                               ///< Set motor RPM.
    COMM_SET_POS,                               ///< Set motor position.
    COMM_SET_HANDBRAKE,                         ///< Set handbrake current.
    COMM_SET_DETECT,                            ///< Set detect mode (e.g., for motor detection).
    COMM_SET_SERVO_POS,                         ///< Set servo position.
    COMM_SET_MCCONF,                            ///< Set motor configuration.
    COMM_GET_MCCONF,                            ///< Get motor configuration.
    COMM_GET_MCCONF_DEFAULT,                    ///< Get default motor configuration.
    COMM_SET_APPCONF,                           ///< Set application configuration.
    COMM_GET_APPCONF,                           ///< Get application configuration.
    COMM_GET_APPCONF_DEFAULT,                   ///< Get default application configuration.
    COMM_SAMPLE_PRINT,                          ///< Request a sample print (debug).
    COMM_TERMINAL_CMD,                          ///< Send a terminal command.
    COMM_PRINT,                                 ///< Print message (debug).
    COMM_ROTOR_POSITION,                        ///< Get rotor position data.
    COMM_EXPERIMENT_SAMPLE,                     ///< Get experiment sample data.
    COMM_DETECT_MOTOR_PARAM,                    ///< Detect motor parameters.
    COMM_DETECT_MOTOR_R_L,                      ///< Detect motor resistance and inductance.
    COMM_DETECT_MOTOR_FLUX_LINKAGE,             ///< Detect motor flux linkage.
    COMM_DETECT_ENCODER,                        ///< Detect encoder parameters.
    COMM_DETECT_HALL_FOC,                       ///< Detect Hall sensor configuration for FOC.
    COMM_REBOOT,                                ///< Reboot the VESC.
    COMM_ALIVE,                                 ///< Keep-alive packet.
    COMM_GET_DECODED_PPM,                       ///< Get decoded PPM signal values.
    COMM_GET_DECODED_ADC,                       ///< Get decoded ADC signal values.
    COMM_GET_DECODED_CHUK,                      ///< Get decoded Nunchuk signal values.
    COMM_FORWARD_CAN,                           ///< Forward CAN message.
    COMM_SET_CHUCK_DATA,                        ///< Set Nunchuk data.
    COMM_CUSTOM_APP_DATA,                       ///< Send/receive custom application data.
    COMM_NRF_START_PAIRING,                     ///< Start NRF (Nordic RF) pairing.
    COMM_GPD_SET_FSW,                           ///< GPD (General Purpose DSP) set FSW.
    COMM_GPD_BUFFER_NOTIFY,                     ///< GPD buffer notification.
    COMM_GPD_BUFFER_SIZE_LEFT,                  ///< GPD get remaining buffer size.
    COMM_GPD_FILL_BUFFER,                       ///< GPD fill buffer with data.
    COMM_GPD_OUTPUT_SAMPLE,                     ///< GPD output sample.
    COMM_GPD_SET_MODE,                          ///< GPD set mode.
    COMM_GPD_FILL_BUFFER_INT8,                  ///< GPD fill buffer with int8 data.
    COMM_GPD_FILL_BUFFER_INT16,                 ///< GPD fill buffer with int16 data.
    COMM_GPD_SET_BUFFER_INT_SCALE,              ///< GPD set buffer integer scale.
    COMM_GET_VALUES_SETUP,                      ///< Request VESC setup values.
    COMM_SET_MCCONF_TEMP,                       ///< Set temporary motor configuration.
    COMM_SET_MCCONF_TEMP_SETUP,                 ///< Set temporary motor configuration for setup.
    COMM_GET_VALUES_SELECTIVE,                  ///< Get selected VESC status values.
    COMM_GET_VALUES_SETUP_SELECTIVE,            ///< Get selected VESC setup values.
    COMM_EXT_NRF_PRESENT,                       ///< Check if external NRF module is present.
    COMM_EXT_NRF_ESB_SET_CH_ADDR,               ///< External NRF ESB set channel and address.
    COMM_EXT_NRF_ESB_SEND_DATA,                 ///< External NRF ESB send data.
    COMM_EXT_NRF_ESB_RX_DATA,                   ///< External NRF ESB receive data.
    COMM_EXT_NRF_SET_ENABLED,                   ///< External NRF set enabled state.
    COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP,    ///< Detect motor flux linkage in open loop.
    COMM_DETECT_APPLY_ALL_FOC,                  ///< Apply all detected FOC parameters.
    COMM_JUMP_TO_BOOTLOADER_ALL_CAN,            ///< Command all VESCs on CAN to jump to bootloader.
    COMM_ERASE_NEW_APP_ALL_CAN,                 ///< Command all VESCs on CAN to erase application memory.
    COMM_WRITE_NEW_APP_DATA_ALL_CAN,            ///< Command all VESCs on CAN to write new application data.
    COMM_PING_CAN,                              ///< Ping VESCs on CAN bus.
    COMM_APP_DISABLE_OUTPUT,                    ///< Disable application output.
    COMM_TERMINAL_CMD_SYNC,                     ///< Synchronous terminal command.
    COMM_GET_IMU_DATA,                          ///< Get IMU data.
    COMM_BM_CONNECT,                            ///< Battery Management System connect.
    COMM_BM_ERASE_FLASH_ALL,                    ///< Battery Management System erase all flash.
    COMM_BM_WRITE_FLASH,                        ///< Battery Management System write to flash.
    COMM_BM_REBOOT,                             ///< Battery Management System reboot.
    COMM_BM_DISCONNECT                          ///< Battery Management System disconnect.
};

#endif // DATATYPES_H
