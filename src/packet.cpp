#include "packet.h"

/// @brief Construct a packet.
/// @param dest_address the address of the destination device
/// @param src_address the address of the source device
/// @param msg_type the type of message
/// @param packet_id the unique id of the packet
/// @param data the payload
/// @param len the length of the payload (must be <= MAX_BUF_LEN)
/// @return a `Packet_t` object
Packet_t make_packet(uint8_t dest_address,
                     uint8_t src_address,
                   Message_t msg_type,
                     uint8_t packet_id,
                    uint8_t* data,
                     uint8_t length) {
    Packet_t packet;
    packet.header.fields.dest_address = dest_address;
    packet.header.fields.src_address  = src_address;
    packet.header.fields.msg_type     = msg_type;
    packet.header.fields.packet_id    = packet_id;
    packet.length                     = length;
    packet.nbytes                     = length + 3;
    memcpy(&packet.payload, data, length);
    return packet;
}

// accessors from the raw data
uint8_t get_dest_address(uint8_t* data) {
    Header header; memcpy(&header, data, sizeof(Header));
    return header.fields.dest_address;
}

uint8_t get_src_address(uint8_t* data) {
    Header header; memcpy(&header, data, sizeof(Header));
    return header.fields.src_address;
}

Message_t get_msg_type(uint8_t* data) {
    Header header; memcpy(&header, data, sizeof(Header));
    return header.fields.msg_type;
}

uint8_t get_packet_id(uint8_t* data) {
    Header header; memcpy(&header, data, sizeof(Header));
    return header.fields.packet_id;
}