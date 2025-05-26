/*
    Copyright 2016 - 2021 Benjamin Vedder	benjamin@vedder.se

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

#ifndef PACKET_H
#define PACKET_H


#include "libvesc/datatypes.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <functional>
#include <libserial/SerialPort.h>
#include "libvesc/utils.h"


namespace {
    using std::vector;
    using std::function;
    using std::round;
    using LibSerial::SerialPort;
    using std::is_integral_v;

    constexpr uint32_t u24{24};
    constexpr uint32_t u16{16};
    constexpr uint32_t u8{8};
    constexpr uint8_t FF = 0xFF;
}

namespace vesc {
    using utils::castu8;
    using utils::castu16;
    using utils::castu32;
    using utils::castdouble;

    /**
     * @brief Minimum total size of a valid VESC packet (including start, length, CRC, and end bytes).
     */
    constexpr uint8_t minTotalPacketSize = 6;

    /**
     * @brief Represents a VESC communication packet.
     * 
     * This class handles the creation and processing of VESC packets. It includes functionality
     * for constructing packets with various data types, calculating CRC checksums,
     * and parsing incoming byte streams into structured packet data.
     * Template methods are defined in `packet.tcc`.
     */
    class Packet {

    public:
        /**
         * @brief Default constructor. Initializes a packet with an undefined state.
         */
        Packet();

        /**
         * @brief Constructs a packet with a specific command ID.
         * @param Id The communication packet ID (`COMM_PACKET_ID`).
         */
        Packet(COMM_PACKET_ID Id);

        /**
         * @brief Constructs a packet with a command ID and initial data.
         * 
         * This constructor is templated to accept various data types. The data is appended
         * to the packet's raw data buffer.
         * @tparam T The type of the initial data to append.
         * @param Id The communication packet ID (`COMM_PACKET_ID`).
         * @param data The initial data to be appended to the packet.
         */
        template<class T>
        Packet(COMM_PACKET_ID Id, T data);

        /**
         * @brief Constructs a packet with a command ID, a double value, and a scale factor.
         * 
         * The double value is scaled and appended as a 32-bit integer.
         * @param Id The communication packet ID (`COMM_PACKET_ID`).
         * @param number The double value to append.
         * @param scale The scale factor to apply to the number before converting to int32.
         */
        Packet(COMM_PACKET_ID Id, double number, double scale);

        /**
         * @brief Defaulted destructor.
         */
        ~Packet();

        /**
         * @brief Creates the final byte vector for sending over a serial connection.
         * 
         * This method takes the accumulated raw data, prepends the length and type,
         * appends the CRC checksum, and the end byte.
         * @return A `std::vector<uint8_t>` containing the complete packet bytes.
         * @note The method is marked [[nodiscard]] to encourage checking the return value.
         */
        [[nodiscard]] vector<uint8_t> createPacket() const;

        /**
         * @brief Calculates the CRC-16 checksum for a given payload.
         * @param payload A constant reference to a vector of bytes for which to calculate the CRC.
         *                  Can also accept `std::span<const uint8_t>` as of C++20 (internal implementation detail).
         * @return The calculated 16-bit CRC checksum.
         */
        static unsigned short crc16(const vector<uint8_t> &payload);

        /**
         * @brief Processes an incoming data buffer to parse a VESC packet.
         * 
         * This method implements a state machine to detect packet length, read the message,
         * calculate and validate the CRC, and determine if a complete, valid packet has been received.
         * @param inputData A vector of bytes representing the incoming data stream.
         *                  This data is processed, and internal state is updated.
         */
        void processData(vector<uint8_t> inputData);

        /**
         * @brief Appends data of a generic type to a message buffer.
         * 
         * This static template method serializes the given data into bytes and appends it
         * to the provided message vector. It handles various integral and floating-point types.
         * Implementations are in `packet.tcc`.
         * @tparam T The type of data to append.
         * @param message The vector of bytes to which the serialized data will be appended.
         * @param data The data item to append.
         */
        template<class T>
        static void append(vector<uint8_t> &message, T data);

        /**
         * @brief Pops (extracts and removes) data of a generic type from the front of a message buffer.
         * 
         * This static template method deserializes data of type T from the beginning of the
         * message vector and removes the consumed bytes from the vector.
         * Implementations are in `packet.tcc`.
         * @tparam T The type of data to pop.
         * @param message The vector of bytes from which data will be popped. Modified by removal of data.
         * @param data Output parameter; the deserialized data item will be stored here.
         */
        template<class T>
        static void pop(vector<uint8_t> &message, T &data);

        /**
         * @brief Appends a double value to a message buffer, scaled and converted to a 32-bit integer.
         * @param message The vector of bytes to append to.
         * @param number The double value to append.
         * @param scale The scale factor to apply before conversion to int32.
         */
        static void appendDouble32(vector<uint8_t> &message, double number, double scale);

        /**
         * @brief Pops a 16-bit integer from a message buffer and converts it to a scaled double.
         * @param message The vector of bytes to pop from.
         * @param scale The scale factor to apply after conversion from int16.
         * @return The deserialized and scaled double value.
         */
        static double popDouble16(vector<uint8_t> &message, double scale);

        /**
         * @brief Pops a 32-bit integer from a message buffer and converts it to a scaled double.
         * @param message The vector of bytes to pop from.
         * @param scale The scale factor to apply after conversion from int32.
         * @return The deserialized and scaled double value.
         */
        static double popDouble32(vector<uint8_t> &message, double scale);

        /**
         * @brief Gets a reference to the processed payload data of the packet.
         * @return A reference to a vector of bytes representing the packet's payload.
         */
        vector<uint8_t> &getPayload();

        /**
         * @brief Gets the minimum total size a VESC packet can have.
         * @return The minimum total packet size as a long integer.
         */
        static long getminTotalPacketSize() { return minTotalPacketSize; }

        /**
         * @brief Checks if the packet has been successfully processed and is considered valid.
         * @return True if the packet is good (CRC validated, end byte correct), false otherwise.
         */
        bool isGoodPacket() { return m_ProcessState == GoodPacket; }


    private:
        /**
         * @brief Buffer storing the raw data (typically command ID and parameters) before packet creation.
         */
        vector<uint8_t> rawData;
        /**
         * @brief Buffer storing the payload of a received and processed packet.
         */
        vector<uint8_t> payload;

        /**
         * @brief Internal states for the packet processing state machine.
         */
        enum States {
            DetectLength = 0,   ///< Initial state, waiting to detect packet length type.
            Length1byte = 2,    ///< Packet type indicates payload length is 1 byte. Corresponds to VESC COMM_SHORT_PACKET.
            Length2byte = 3,    ///< Packet type indicates payload length is 2 bytes. Corresponds to VESC COMM_LONG_PACKET.
            Length3byte = 4,    ///< Packet type indicates payload length is 4 bytes. Corresponds to VESC COMM_EXTENDED_PACKET (hypothetical, VESC firmware uses 1 or 2 bytes for payload length in type byte 2 and 3).
            ReadMessage = 5,    ///< State for reading the main message body (payload).
            CalcCRC = 6,        ///< State for calculating CRC of the received payload. (Often integrated into ReadMessage or ValidateCRC)
            ValidateCRC = 7,    ///< State for validating the received CRC against the calculated one.
            GoodPacket = 8      ///< State indicating a complete and valid packet has been processed.
        };

        /**
         * @brief The current state of the packet processing state machine.
         */
        States m_ProcessState;

    };

#include "packet.tcc" // Template implementations
}
#endif // PACKET_H
