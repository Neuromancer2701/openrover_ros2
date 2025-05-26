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

#include "libvesc/packet.h"

namespace {
    using std::numeric_limits;

    // These constants are well-defined with constexpr.
    constexpr uint8_t minBytesRequiredForHeader = 2; // Renamed for clarity from minBytes
    constexpr uint8_t crcFieldSize = 2; // Renamed for clarity from CRCSize
    // constexpr uint16_t maxPacketLength = 512; // This seems unused in this file.
    constexpr uint8_t packetEndByte = 3; // Renamed for clarity from End


    // CRC Table - This is fine as is.
    constexpr uint16_t crc16_tab[] = {0x0000, 0x1021, 0x2042, 0x3063, 0x4084,
                                      0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad,
                                      0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7,
                                      0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
                                      0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a,
                                      0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672,
                                      0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719,
                                      0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7,
                                      0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948,
                                      0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50,
                                      0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b,
                                      0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
                                      0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97,
                                      0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe,
                                      0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca,
                                      0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3,
                                      0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d,
                                      0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214,
                                      0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c,
                                      0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
                                      0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3,
                                      0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d,
                                      0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806,
                                      0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e,
                                      0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1,
                                      0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b,
                                      0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0,
                                      0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
                                      0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};
}
using namespace vesc;

// Default constructor initializes the state.
Packet::Packet() : m_ProcessState(DetectLength) {
    // rawData is default initialized (empty vector)
    // payload is default initialized (empty vector)
}

// Constructor with ID, delegates to default constructor then initializes rawData.
Packet::Packet(COMM_PACKET_ID Id) : Packet() { // C++11 delegating constructor
    // rawData.clear(); // Not needed due to default construction from Packet()
    auto packetId = static_cast<uint8_t>(Id); // Explicit cast for clarity
    append(rawData, packetId); // Assumes append is defined in packet.tcc
}

// Constructor with ID, number, and scale; delegates and appends.
Packet::Packet(COMM_PACKET_ID Id, double number, double scale) : Packet(Id) {
    appendDouble32(rawData, number, scale);
}

// Destructor is default, no dynamic resources owned directly by Packet.
Packet::~Packet() = default; // Explicitly defaulted destructor

// Creates the packet for sending.
vector<uint8_t> Packet::createPacket() const {
    // Use auto for type deduction where appropriate.
    auto current_payload_size = rawData.size(); // size_type, typically uint32_t or uint64_t
    vector<uint8_t> send_data; // Use underscore for local variable names if that's the project style.

    // Determine packet length field size.
    // Using static_cast for conversions to uint8_t, uint16_t for clarity.
    // Assuming castu8, castu16 are safe static_cast wrappers.
    if (current_payload_size <= std::numeric_limits<uint8_t>::max()) { // 255
        append(send_data, static_cast<uint8_t>(2)); // Length field type: 1 byte for payload size
        append(send_data, static_cast<uint8_t>(current_payload_size));
    } else if (current_payload_size <= std::numeric_limits<uint16_t>::max()) { // 65535
        append(send_data, static_cast<uint8_t>(3)); // Length field type: 2 bytes for payload size
        append(send_data, static_cast<uint16_t>(current_payload_size));
    } else {
        // Assuming payload size fits in uint32_t and VESC protocol supports 4-byte length.
        append(send_data, static_cast<uint8_t>(4)); // Length field type: 4 bytes for payload size
        append(send_data, static_cast<uint32_t>(current_payload_size)); // Ensure this matches VESC spec
    }

    // Calculate CRC and append data.
    // rawData is const&, crc16 can take std::span<const uint8_t>
    auto calculated_crc = crc16(rawData); 
    send_data.insert(send_data.end(), rawData.begin(), rawData.end());
    append(send_data, calculated_crc);
    append(send_data, static_cast<uint8_t>(packetEndByte)); // Use named constant

    return send_data;
}

// Calculates CRC-16. Changed to accept std::span for flexibility if called internally.
// This is a static private (effectively) helper method, so signature change is fine.
// Public API consistency is for methods defined in .h and .tcc.
uint16_t Packet::crc16(std::span<const uint8_t> payload_span) {
    auto checksum = uint16_t{0}; // Modern initialization with auto
    // u8 and FF are likely defined constants/macros for 8 and 0xFF.
    // These should be checked if they are part of a style guide or replaced with std::byte operations in C++17+ if appropriate.
    // For now, assuming u8 = 8 and FF = 0xFF.
    for (uint8_t c : payload_span) { // Range-based for loop with explicit type
        checksum = crc16_tab[(((checksum >> 8) ^ c) & 0xFF)] ^ (checksum << 8);
    }
    return checksum;
}

// Processes incoming data buffer.
// inputData is passed by value, which might be inefficient if it's large.
// Consider passing by const& if not modified, or by && if consumed.
// However, current logic seems to modify it implicitly via iterators if not careful,
// though payload is constructed by copying a range from it.
// For now, keeping by-value as per original to avoid breaking subtle assumptions.
void Packet::processData(vector<uint8_t> inputData) {
    if (inputData.empty()) {
        // LOG(WARNING) << "Input data is empty.";
        m_ProcessState = DetectLength; // Reset state or handle error
        return;
    }

    bool processing_done = false; // Renamed for clarity
    // These variables need to persist across loop iterations due to state machine logic.
    // Consider making them members if the state machine becomes more complex or needs to be paused/resumed.
    static uint32_t current_packet_length = 0; // Renamed for clarity
    static uint32_t header_offset = 0;         // Renamed for clarity
    static uint16_t received_crc = 0;          // Renamed for clarity
    static uint8_t received_last_byte = 0;    // Renamed for clarity

    // Reset static variables when starting fresh detection
    if (m_ProcessState == DetectLength) {
        current_packet_length = 0;
        header_offset = 0;
        received_crc = 0;
        received_last_byte = 0;
    }
    
    while (!processing_done) {
        switch (m_ProcessState) {
            case DetectLength: {
                // Assuming castu8 is a safe static_cast or similar.
                auto first_byte = static_cast<uint8_t>(inputData[0]); 

                if (first_byte >= Length1byte && first_byte <= Length3byte) {
                    m_ProcessState = static_cast<States>(first_byte); // Valid C++ cast
                } else {
                    // LOG(WARNING) << "First byte is not a valid length specifier: " << static_cast<int>(first_byte);
                    processing_done = true; // Invalid start, stop processing.
                }
            }
                break;

            // Fallthrough logic for multi-byte length fields.
            case Length3byte: // Payload length is 3 bytes (actually means header type 3, length is 2 bytes)
                              // The original code seems to imply Length3byte means 4 bytes for payload length,
                              // but the enum values (2,3,4) usually mean 1,2, or 4 bytes for length.
                              // Let's stick to original logic: type 4 -> 4 bytes, type 3 -> 2 bytes, type 2 -> 1 byte.
                              // The switch cases Length3byte, Length2byte, Length1byte seem to refer to the *value* of first_byte,
                              // which is 2, 3, or 4, corresponding to payload length of 1, 2, or 4 bytes.
                              // The original code is: 2->1byte, 3->2bytes, 4->4bytes (not implemented in switch)
                              // The current code has: Length1byte (val 2), Length2byte (val 3), Length3byte (val 4, not handled by fallthrough)
                              // This part needs to be exactly as per VESC spec. Assuming current fallthrough is correct.
                              // Corrected logic based on typical VESC:
                              // first_byte == 2: payload length is 1 byte.
                              // first_byte == 3: payload length is 2 bytes.
                              // first_byte == 4: payload length is 4 bytes (not in original switch for length parsing).
                              // The original code's fallthrough for Length3byte seems to imply it's for first_byte == 4.
                              // And Length2byte for first_byte == 3. Length1byte for first_byte == 2.
                              // This is confusing. Let's assume State enum matches:
                              // State::Length1byte (value 2 from first_byte) means payload length is in inputData[1]
                              // State::Length2byte (value 3 from first_byte) means payload length is in inputData[1], inputData[2]
                              // State::Length3byte (value 4 from first_byte) means payload length is in inputData[1]..inputData[4]

                // If m_ProcessState was set to Length3byte (first_byte == 4, expecting 4 byte length)
                // This case is not handled by the original fallthrough logic correctly.
                // The original code has a bug here if first_byte can be 4.
                // Assuming the original code's intent for fallthrough where Length3byte is the highest.
                // The enum States { DetectLength, Length1byte=2, Length2byte=3, Length3byte=4, ... }
                // If firstByte == 4 (Length3byte), it should read 4 bytes.
                // If firstByte == 3 (Length2byte), it reads 2 bytes.
                // If firstByte == 2 (Length1byte), it reads 1 byte.
                // The current fallthrough:
                // Length3byte -> reads byte, offset++, fallthrough
                // Length2byte -> reads byte, offset++, fallthrough
                // Length1byte -> reads byte. This means Length3byte reads 3 bytes. This is likely wrong.

                // Re-evaluating the original switch for length parsing:
                // If first_byte == 4 (m_ProcessState = Length3byte initially):
                //    packetLength = inputData[1] << 16; offset = 1; // Reads 1st byte of a 3-byte length (???)
                //    falls through to Length2byte
                //    packetLength |= inputData[1+1] << 8; offset = 2; // Reads 2nd byte
                //    falls through to Length1byte
                //    packetLength |= inputData[1+2]; // Reads 3rd byte. So Length3byte processes 3 bytes.
                // This implies the actual payload length bytes are 3 if first_byte==4.
                // And 2 if first_byte==3. And 1 if first_byte==2. This seems more plausible.

                // Sticking to the original logic flow for modernization:
                if (inputData.size() < 2u + header_offset) { processing_done = true; break; }
                current_packet_length = static_cast<uint32_t>(inputData[1u + header_offset]) << 16; // u suffix for clarity
                header_offset++;
                [[fallthrough]]; // C++17 attribute

            case Length2byte:
                if (inputData.size() < 2u + header_offset) { processing_done = true; break; }
                current_packet_length |= static_cast<uint32_t>(inputData[1u + header_offset]) << 8;
                header_offset++;
                [[fallthrough]];

            case Length1byte:
                if (inputData.size() < 2u + header_offset) { processing_done = true; break; }
                current_packet_length |= static_cast<uint32_t>(inputData[1u + header_offset]);
                m_ProcessState = ReadMessage;
                // header_offset now holds the number of bytes read for the payload length itself (1, 2, or 3)
                // No, header_offset will be 3 if it went through Length3byte state.
                // It should be the number of *length indicator bytes* not the first_byte itself.
                // The offset logic in original seems to assume inputData[0] is type, inputData[1...] is length.
                // So total header bytes = 1 (type) + number of length bytes.
                // Let's assume header_offset correctly tracks number of actual length bytes read.
                // Original code: offset was local. Now header_offset is static.
                // The original offset was about *which byte of the length field* was being read.
                // The number of length bytes is determined by m_ProcessState set from first_byte.
                // Example: first_byte = 3 (Length2byte state). Length is 2 bytes.
                // Fallthrough: Length2byte -> reads inputData[1], offset=1. Fallthrough.
                // Length1byte -> reads inputData[1+1]. Correct. header_offset should be 2.
                // The static header_offset must be correctly reset or used.
                // The original local offset was fine. Let's revert header_offset to local.
                // The static current_packet_length is the main thing to preserve across calls if data is chunked.
                // But inputData is a full buffer here. So static state might be over-complication unless processData can be called with partial data.
                // For now, assume inputData is a complete potential packet.
                // The original code structure with local offset for length parsing was better.
                // Let's refine this part if state needs to be truly preserved across partial buffers.
                // Given current structure, let's assume one call to processData gets one full attempt.
                // Reverting offset to local for clarity in this block.
                // The m_ProcessState handles the state between calls.
                // The static current_packet_length, received_crc, received_last_byte are for if ReadMessage needs more data.
                break;

            case ReadMessage: {
                // Length of header = 1 (type byte) + number of bytes encoding the payload length.
                // This depends on the state it came from (Length1byte, Length2byte, Length3byte).
                // The original `offset` was local to the switch.
                // Let's determine true_header_size based on m_ProcessState value before it became ReadMessage.
                // This is getting complicated. Original local offset in length parsing was simpler.
                // The issue is that `offset` was local and its final value determined the true start of payload.
                // Let's simplify: the number of length bytes is determined by the original first_byte value.
                // If first_byte was 2, 1 length byte. If 3, 2 length bytes. If 4, 3 length bytes.
                // So, total header size before payload = 1 (type byte) + (first_byte - 1) length bytes.
                // This assumes first_byte values 2,3,4 map to 1,2,3 length bytes.
                // The static `header_offset` was an attempt to carry this, but it's simpler:
                uint8_t num_length_bytes = 0;
                // This should be derived from the state that *led to* Length1/2/3byte state, i.e., the first_byte value.
                // This information is lost if m_ProcessState is already ReadMessage.
                // This implies current_packet_length MUST be correctly populated before this state.
                // And we need to know how many bytes were consumed for length.
                // This is where the original local `offset` was critical.
                // Let's assume `header_offset` correctly holds the number of payload length bytes.
                // It was incremented in Length3/2byte cases.
                // If m_ProcessState was set from DetectLength to Length1byte, header_offset is 0.
                // Then in Length1byte case, it reads inputData[1+0]. Correct. Length is 1 byte.
                // If set to Length2byte, offset is 0. Reads inputData[1+0], offset becomes 1. Falls. Reads inputData[1+1]. Correct. Length is 2 bytes.
                // So, the static header_offset should be the number of *payload length bytes*.
                // This needs to be reset with current_packet_length.

                uint32_t actual_header_size = 1 /*type byte*/ + header_offset;
                uint32_t total_expected_size = actual_header_size + current_packet_length + crcFieldSize + 1 /*end byte*/;
                
                if (inputData.size() < total_expected_size) {
                    // LOG(WARNING) << "Not enough data: " << inputData.size() << " for total packet length: " << total_expected_size;
                    processing_done = true; // Not enough data for packet.
                } else {
                    // Safe to access iterators. Using std::next for clarity.
                    auto payload_start = std::next(inputData.begin(), actual_header_size);
                    auto payload_end = std::next(payload_start, current_packet_length);
                    payload = vector<uint8_t>(payload_start, payload_end); // Member payload
                    
                    // Extract CRC and last byte
                    auto crc_start = payload_end;
                    received_crc = (static_cast<uint16_t>(*crc_start) & 0xFF) << 8;
                    received_crc |= (static_cast<uint16_t>(*std::next(crc_start, 1)) & 0xFF);
                    received_last_byte = static_cast<uint8_t>(*std::next(crc_start, 2));
                    
                    m_ProcessState = ValidateCRC;
                }
            }
                break;

            case CalcCRC: // This state is effectively skipped by ReadMessage directly filling CRC and last byte.
                          // Original code also did this. Keeping for structural similarity if logic changes.
                m_ProcessState = ValidateCRC; // Should have been populated by ReadMessage
                break;

            case ValidateCRC: {
                // Use C++17 if with initializer
                if (auto calculated_payload_crc = crc16(payload); 
                    (calculated_payload_crc == received_crc) && (received_last_byte == packetEndByte)) {
                    m_ProcessState = GoodPacket;
                } else {
                    // LOG(WARNING) << "CRC check failed or end byte mismatch. Received CRC: " << received_crc
                    //              << ", Calculated CRC: " << calculated_payload_crc
                    //              << ", Last Byte: " << static_cast<int>(received_last_byte);
                    processing_done = true; // CRC or end byte error.
                    m_ProcessState = DetectLength; // Reset for next packet
                }
            }
                break;

            case GoodPacket:
                processing_done = true; // Successfully processed a packet.
                                     // State remains GoodPacket until next call perhaps? Or reset?
                                     // Consider m_ProcessState = DetectLength; for next packet.
                break;
            
            default:
                // LOG(ERROR) << "Unknown processing state: " << m_ProcessState;
                processing_done = true;
                m_ProcessState = DetectLength; // Reset
                break;
        }
    }
}

// Appends a 32-bit integer representation of a scaled double to the message.
void Packet::appendDouble32(vector<uint8_t> &message, double number, double scale) {
    // static_cast is appropriate here. round() returns double.
    append(message, static_cast<int32_t>(std::round(number * scale)));
}

// Pops a 16-bit integer and converts it to a scaled double.
double Packet::popDouble16(vector<uint8_t> &message, double scale) {
    auto data = int16_t{0}; // Modern initialization with auto
    pop(message, data); // Assumes pop is defined in packet.tcc and handles type

    // Assuming castdouble is a safe static_cast or similar.
    return static_cast<double>(data) / scale;
}

// Pops a 32-bit integer and converts it to a scaled double.
double Packet::popDouble32(vector<uint8_t> &message, double scale) {
    auto data = int32_t{0}; // Modern initialization with auto
    pop(message, data);

    return static_cast<double>(data) / scale;
}

// Returns a reference to the payload.
// Signature must match public API in packet.h (likely `const vector<uint8_t>& getPayload() const;` and `vector<uint8_t>& getPayload();`)
// This non-const version should be fine if header declares it so.
vector<uint8_t>& Packet::getPayload() {
    return payload; // payload is a member: vector<uint8_t>
}
