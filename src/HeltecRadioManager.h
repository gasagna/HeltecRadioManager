#ifndef _HELTECRADIOMANAGER_H
#define _HELTECRADIOMANAGER_H

#include "LoRaWan_APP.h"
#include "packet.h"


/// @brief the state of the ::RadioManager object
typedef enum {
      MANAGER_TX, ///< the manager is trasmitting
      MANAGER_RX, ///< the manager is receiving
            IDLE, ///< the manager is idle
        LOWPOWER, ///< the manager is in low power state
        HAS_RECV, ///< manager has received a packet
        HAS_SENT, ///< manager has finished sending a packet and radio is in rx mode
    HAS_TIMEDOUT, ///< manager has timedout
    HAS_SENT_ACK, ///< manager has sent an acknowledgment
    HAS_RECV_ACK  ///< manager has received an acknowledgment to a packet it sent
} RadioManagerState;

// for pretty printing
#ifdef DEBUGRADIO
    const char STATEMAP[9][13] = {"MANAGER_TX",
                                  "MANAGER_RX",
                                  "IDLE",
                                  "LOWPOWER",
                                  "HAS_RECV",
                                  "HAS_SENT",
                                  "HAS_TIMEOUT",
                                  "HAS_SENT_ACK",
                                  "HAS_RECV_ACK"};
#endif

class RadioManagerClass {
    public:
        bool begin(uint8_t address,
                  uint32_t frequency,
                    int8_t power,
                  uint32_t bandwidth = 0,
                  uint32_t datarate = 7,
                   uint8_t coderate = 1,
                  uint32_t bandwidthAfc = 0,
                  uint16_t preambleLen = 8,
                      bool fixLen = false,
                   uint8_t payloadLen = 0,
             RadioModems_t modem = MODEM_LORA,
                      bool crcOn = true,
                      bool FreqHopOn = false,
                   uint8_t HopPeriod = 0,
                      bool iqInverted = false,
                  uint32_t fdev = 0,
                  uint16_t symbTimeout = 0,
                  uint32_t timeout = 1000,
                      bool rxContinuous = true);
    private:
        void _reset_events();

    public:
        uint8_t getAddress();
        uint8_t getLastPacketID();
           void setMaxRetries(uint16_t max_retries);
           bool send(uint8_t* data, uint8_t length, uint8_t dest_address, bool confirmed, uint32_t ack_timeout_ms, int16_t* msg_rssi = NULL, int16_t* msg_snr = NULL, int16_t* ack_rssi = NULL, int16_t* ack_snr = NULL);
           void recv(uint8_t* data, uint8_t* length, uint8_t* from, int16_t* rssi, int16_t* snr);
           void storePacket(uint8_t *packet, uint16_t length, int16_t rssi, int8_t snr);

    private:
        RadioEvents_t                _events;                        ///<
        uint16_t                     _max_retries;                   ///<
        uint8_t                      _address;                       ///<
        uint8_t                      _last_packet_id;                ///<
        Packet_t                     _packet;                        ///< temporary variable for passign data around between callbacks and other functions

    public:
        volatile RadioManagerState   state;                          ///< the state of the radio. Defined in radio.h

};


// callback functions
void _send_onRxDone_ack(uint8_t *payload, uint16_t length, int16_t rssi, int8_t snr);
void _send_onRxTimeout_ack();
void _send_onRxError_ack();
void _send_onTxDone_ack();
void _send_onTxDone();
void _send_onTxTimeout();

void _recv_onRxDone(uint8_t *payload, uint16_t length, int16_t rssi, int8_t snr);
void _recv_onTxDone();
void _recv_onRxError();

// this become available in sketches
extern RadioManagerClass RadioManager;

#endif