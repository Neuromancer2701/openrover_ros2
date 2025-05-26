

#include <string>
#include <string_view>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "libvesc/Vesc.h"
#include "libvesc/commands.h"


namespace {
    using std::to_string;
    using std::this_thread::sleep_for;
    using std::chrono::milliseconds;
    using std::all_of;
    using LibSerial::DataBuffer; // Assuming DataBuffer is from LibSerial
}

// Using namespace for vesc members is fine within the .cpp file
using namespace vesc; 

// Anonymous namespace for internal linkage helper functions
namespace {
    inline void sleep_ms(long long millis) { 
        std::this_thread::sleep_for(std::chrono::milliseconds(millis)); 
    }
}

Vesc::Vesc(rclcpp::Logger logger)
        : logger_(logger), m_VescIDPacket(Commands::getVescIDpacket()), m_KeepAlivePacket(Commands::getKeepAlivepacket()),
          m_AllMotorDataPacket(Commands::getMotorpacket()), m_SelectmotorDataPacket(Commands::getSelectMotorpacket()),
          m_RPMPacket(Commands::getMotorRPMpacket()), wheel_found({{left_back,   false},
                                                                   {left_front,  false},
                                                                   {right_back,  false},
                                                                   {right_front, false}}) {
}


bool Vesc::isTwoWheelDrive() {
    return ((wheel_found[left_back] && wheel_found[right_back]) ||
            (wheel_found[left_front] && wheel_found[right_front]));
}

bool Vesc::isFourWheelDrive() {
    return all_of(begin(wheel_found), end(wheel_found), [](auto key_value) { return key_value.second; });
}

void Vesc::SetWheelsRPM(unordered_map<int, int> wheel_rpms) {
    for (auto const& [id, port_str] : wheel_ports) { // Use const& for map iteration
        if (auto it = wheel_rpms.find(id); it != wheel_rpms.end()) {
            SerialPort vescPort(port_str); // port_str is std::string
            if (vescPort.IsOpen()) { // Check if port opened successfully
                vescPort.Write(Packet(COMM_SET_RPM, static_cast<unsigned>(it->second)).createPacket());
                vescPort.Close(); // Close port after use
            } else {
                RCLCPP_WARN(logger_, "Could not open port: %s for ID: %d", port_str.c_str(), id);
            }
        } else {
            RCLCPP_WARN(logger_, "ID Not Found in input RPM set data: %d", id);
        }
        sleep_ms(5); // Consider if this sleep is needed per wheel or after all operations
    }
}

void Vesc::SetWheelsDuty(unordered_map<int, double> wheel_duty) {
    for (auto const& [id, port_str] : wheel_ports) { // Use const& for map iteration
        if (auto it = wheel_duty.find(id); it != wheel_duty.end()) {
            SerialPort vescPort(port_str); // port_str is std::string
            if (vescPort.IsOpen()) { // Check if port opened successfully
                vescPort.Write(Packet(COMM_SET_DUTY, it->second, 1e5).createPacket());
                vescPort.Close(); // Close port after use
            } else {
                RCLCPP_WARN(logger_, "Could not open port: %s for ID: %d", port_str.c_str(), id);
            }
        } else {
            RCLCPP_WARN(logger_, "ID Not Found in input Duty set data: %d", id);
        }
        sleep_ms(5); // Consider if this sleep is needed per wheel or after all operations
    }
}

unordered_map<int, double> Vesc::GetWheelsRPM() {
    unordered_map<int, double> rpm_data;
    for (auto const& [id, port_str] : wheel_ports) { // Use const&
        // Pass port_str (std::string) to SendAndReceive, which now expects string_view
        // std::string is implicitly convertible to std::string_view
        if (SendAndReceive(m_RPMPacket, port_str)) { 
            rpm_data[id] = cmd.getMotorControllerData().rpm;
        }
        sleep_ms(5);
    }
    return rpm_data;
}

void Vesc::FindandMapMotorControllers() {
    SerialPort testport; // Default constructor
    auto serialPorts = testport.GetAvailableSerialPorts(); // Use auto
    vector<string> filteredPorts;
    // Use std::ranges::copy_if with a projection in C++20 for more elegance if available and headers allow
    // For now, keeping std::copy_if
    std::copy_if(serialPorts.begin(), serialPorts.end(), std::back_inserter(filteredPorts),
                 [](const std::string& s) { return s.find("ttyACM") != std::string::npos; });

    for (auto const& port_str : filteredPorts) { // Use const&
        // Pass port_str (std::string) to SendAndReceive
        if (SendAndReceive(m_VescIDPacket, port_str)) { 
            auto id = cmd.getMotorControllerData().vesc_id; // Use auto
            // Using a switch statement here is fine.
            // Consider std::map or other structures if cases become very numerous or complex.
            switch (id) {
                // Cases can be simplified if left_front/right_front share logic with back wheels
                // or if specific handling for them is added later.
                default: // Catches unhandled IDs, could log a warning.
                case left_front:    // Fall-through if behavior is same as left_back for now
                case right_front:   // Fall-through if behavior is same as right_back for now
                case left_back:
                case right_back:
                    // port_str is std::string, wheel_ports stores std::string
                    wheel_ports[id] = port_str; 
                    wheel_found[id] = true;
                    break;
            }
        }
    }
}

// Private helper function, changed to accept std::string_view
// This change is internal and does not affect the public API in Vesc.h
bool Vesc::SendAndReceive(const Packet &packet, std::string_view port_sv) {
    SerialPort vescPort(std::string(port_sv)); // SerialPort constructor might need std::string
    if (vescPort.IsOpen()) {
        vescPort.FlushIOBuffers();
        vescPort.Write(packet.createPacket());
        sleep_ms(25); // Consider making delay configurable or dynamic

        auto bytes_available = vescPort.GetNumberOfBytesAvailable(); // Use auto
        // Loop with a timeout or max attempts could be more robust
        while (bytes_available < Packet::getminTotalPacketSize()) {
            sleep_ms(25); // Re-check after delay
            bytes_available = vescPort.GetNumberOfBytesAvailable();
        }

        DataBuffer buffer; // Assuming DataBuffer is from LibSerial
        vescPort.Read(buffer, bytes_available);
        // vescPort.Close(); // Close port as soon as possible

        Packet incoming; // Default constructor
        incoming.processData(buffer);
        if (incoming.isGoodPacket()) {
            cmd.processPacket(incoming.getPayload());
            // vescPort.Close(); // Ensure port is closed on successful path too
            return true;
        }
        // vescPort.Close(); // Ensure port is closed on packet error path
        return false;  // Packet is not good
    }
    RCLCPP_WARN(logger_, "Could not open port: %s", std::string(port_sv).c_str());
    return false; // couldn't open port
}

// This public API function signature remains unchanged (takes const int&)
// It calls the internal SendAndReceive which now takes string_view
bool Vesc::SendAndReceive(const Packet &packet, const int &port_id) {
    if (auto it = wheel_ports.find(port_id); it != wheel_ports.end()) {
        // it->second is std::string, which is compatible with std::string_view parameter
        return SendAndReceive(packet, it->second); 
    }
    RCLCPP_WARN(logger_, "Port ID not found in wheel_ports: %d", port_id);
    return false;  //could not find specified port
}


unordered_map<int, MC_VALUES> Vesc::GetSelectMotorData() {
    unordered_map<int, MC_VALUES> motor_data;
    for (auto const& [id, port_str] : wheel_ports) { // Use const&
        if (SendAndReceive(m_SelectmotorDataPacket, port_str)) { // Pass std::string
            motor_data[id] = cmd.getMotorControllerData();
        }
        sleep_ms(5);
    }
    return motor_data;
}


unordered_map<int, MC_VALUES> Vesc::GetAllMotorData() {
    unordered_map<int, MC_VALUES> motor_data;
    for (auto const& [id, port_str] : wheel_ports) { // Use const&
        if (SendAndReceive(m_AllMotorDataPacket, port_str)) { // Pass std::string
            motor_data[id] = cmd.getMotorControllerData();
        }
        sleep_ms(5);
    }
    return motor_data;
}
