#ifndef _PACKET_H
#define _PACKET_H

#include "Arduino.h"

#define MAX_PAYLOAD_LEN 0xFF ///< The maximum payload length

/// @brief The type of a message
typedef enum {
        UNCONFIRMED,  ///< the type of messages sent that do not require to be acknowledged
          CONFIRMED,  ///< the type of messages sent that require to be acknowledged
    ACKNOWLEDGEMENT,  ///< the type of messages that acknowledge other messages
} Message_t;

/// @brief A bit field struct to store packet information compactly in 
// as little space as possible, i.e. two bytes. The size in bits of each
// field also determines the range of values that the fields can have.
struct Header {
   uint8_t dest_address : 5; ///< the destination address, from 0 (usually for the gateway) to 31
   uint8_t    packet_id : 3; ///< the packet id, from 0 to 7
   uint8_t  src_address : 5; ///< the address of the source device, from 0 to 31
   Message_t   msg_type : 3; ///< the type of the message, see ::Message_t
} __attribute__((packed)); // this is necessary to force this structure to be 2 bytes

// union Header {
    // uint16_t header;
    // struct HeaderFields fields;
// };

/// @brief Data structure for packets sent with HeltecRadioManager. This is part of the
/// private interface and this type is not exposed to user code.
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

Packet_t make_packet(uint8_t dest_address,
                     uint8_t src_address,
                   Message_t message_type,
                     uint8_t packet_id,
                    uint8_t* data,
                     uint8_t length);

uint8_t get_dest_address(uint8_t* data);
uint8_t get_src_address(uint8_t* data);
Message_t get_msg_type(uint8_t* data);
uint8_t get_packet_id(uint8_t* data);

#endif