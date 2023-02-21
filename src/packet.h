#ifndef _PACKET_H
#define _PACKET_H

#include "Arduino.h"

#define MAX_PAYLOAD_LEN 0xFF ///< The maximum payload length

/// @brief the type of the message
typedef enum {
        UNCONFIRMED,  ///< the type of messages sent that do not require to be acknowledged
          CONFIRMED,  ///< the type of messages sent that require to be acknowledged
    ACKNOWLEDGEMENT,  ///< the type of messages that acknowledge other messages
} Message_t;

// The header packs all info into two bytes
// Order is important to ensure alignment
struct HeaderFields {
   uint8_t dest_address : 5;
   uint8_t    packet_id : 3;
   uint8_t  src_address : 5;
   Message_t   msg_type : 3;
} __attribute__((packed)); // this is necessary to force this structure to be 2 bytes

union Header {
    uint16_t header;
    struct HeaderFields fields;
};

/// @brief Data structure for packets sent with HeltecRadioManager.
typedef struct {
    // this is the part that get sent
    Header  header;                   ///< the header
    uint8_t length;                   ///< the length of the payload
    uint8_t payload[MAX_PAYLOAD_LEN]; ///< the buffer array where data is stored – Not all is sent

    // the fields below are not sent, but are stored in a Packet_t object and are accessible
    int16_t rssi;                     ///< the rssi of the packet just received
    int16_t snr;                      ///< the snr of the packet just received
    uint8_t nbytes;                   ///< the number of bytes of this packet that are actually sent
} Packet_t;

// create packet
Packet_t make_packet(uint8_t dest_address,
                     uint8_t src_address,
                   Message_t message_type,
                     uint8_t packet_id,
                    uint8_t* data,
                     uint8_t length);

// accessor from raw packet bytes
uint8_t get_dest_address(uint8_t* data);
uint8_t get_src_address(uint8_t* data);
Message_t get_msg_type(uint8_t* data);
uint8_t get_packet_id(uint8_t* data);

#endif