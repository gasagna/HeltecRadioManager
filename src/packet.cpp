#include "packet.h"

/// @brief Construct a packet.
/// @param dest_address [IN] the address of the destination device
/// @param src_address [IN] the address of the source device
/// @param msg_type [IN] the type of message
/// @param packet_id [IN] the id of the packet, from 0 to 7
/// @param data [IN] the payload
/// @param len [IN] the length of the payload (must be <= MAX_BUF_LEN)
/// @return a `Packet_t` object
Packet_t make_packet(uint8_t dest_address,
                     uint8_t src_address,
                   Message_t msg_type,
                     uint8_t packet_id,
                    uint8_t* data,
                     uint8_t length) {
    Packet_t packet;
    packet.header.dest_address = dest_address;
    packet.header.src_address  = src_address;
    packet.header.msg_type     = msg_type;
    packet.header.packet_id    = packet_id;
    packet.length              = length;
    packet.nbytes              = length + 3;
    memcpy(&packet.payload, data, length);
    return packet;
}

/// @brief Get destination address from raw LoRa data. Addresses are stored
/// as five bit numbers, so the address ranges from 0 (usually reserved for 
/// gateway) to 31.
/// @param data [IN] pointer to the raw LoRa data
/// @return the destination address
uint8_t get_dest_address(uint8_t* data) {
    Header header; memcpy(&header, data, sizeof(Header));
    return header.dest_address;
}

/// @brief Get source address from raw LoRa data. Addresses are stored
/// as five bit numbers, so the address ranges from 0 (usually reserved for 
/// gateway) to 31.
/// @param data [IN] pointer to the raw LoRa data
/// @return the source address
uint8_t get_src_address(uint8_t* data) {
    Header header; memcpy(&header, data, sizeof(Header));
    return header.src_address;
}

/// @brief Get message type from raw LoRa data. See ::Message_t for details.
/// @param data [IN] pointer to the raw LoRa data
/// @return the message type
Message_t get_msg_type(uint8_t* data) {
    Header header; memcpy(&header, data, sizeof(Header));
    return header.msg_type;
}

/// @brief Get packet id. The id ranges from 0 to 7.
/// @param data [IN] pointer to the raw LoRa data
/// @return the packet id.
uint8_t get_packet_id(uint8_t* data) {
    Header header; memcpy(&header, data, sizeof(Header));
    return header.packet_id;
}