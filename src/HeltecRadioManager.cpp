#include "Arduino.h"
#include "HeltecRadioManager.h"

/// 
void _lowPowerHandler() {
    #if defined(Wireless_Stick_V3)
        // TODO:
    #elif defined(CubeCell_Board) || defined(CubeCell_Board_V2) || \
          defined(CubeCell_Capsule) || defined(CubeCell_Module) || \
          defined(CubeCell_Module_V2)  || defined(CubeCell_BoardPlus) || \
          defined(CubeCell_GPS) || defined(CubeCell_ModulePlus) || \
          defined(CubeCell_BoardPRO)
        lowPowerHandler();
    #endif
}

/*****************************************************************************/
/* macros                                                                    */
/*****************************************************************************/
/// Convert two bytes to an int16_t
#define bytes_to_int16_t(MSB, LSB) (static_cast<int16_t>(MSB << 8 | LSB))

/*****************************************************************************/
/* global variable avaiable in sketches                                      */
/*****************************************************************************/
RadioManagerClass RadioManager = RadioManagerClass();

/*****************************************************************************/
/* begin                                                                     */
/*****************************************************************************/
bool RadioManagerClass::begin( uint8_t address,
                              uint32_t frequency,
                                int8_t power,
                              uint32_t bandwidth,
                              uint32_t datarate,
                               uint8_t coderate,
                              uint32_t bandwidthAfc,
                              uint16_t preambleLen,
                                  bool fixLen,
                               uint8_t payloadLen,
                         RadioModems_t modem,
                                  bool crcOn,
                                  bool FreqHopOn,
                               uint8_t HopPeriod,
                                  bool iqInverted,
                              uint32_t fdev,
                              uint16_t symbTimeout,
                              uint32_t timeout,
                                  bool rxContinuous) {
        // init members
        _max_retries    = 5;
        _address        = address;
        _last_packet_id = 0; // varies between 0 and 7
        state           = RadioManagerState::IDLE;

        // init radio
        Radio.Init(&_events);

        // make sure frequency is supported
        if (Radio.CheckRfFrequency(frequency) == false)
            return false;

        // set radio config parameters
        Radio.SetChannel(frequency);
        Radio.SetRxConfig(modem, bandwidth, datarate, coderate, bandwidthAfc, preambleLen,
                          symbTimeout, fixLen, payloadLen, crcOn, FreqHopOn, HopPeriod,
                          iqInverted, rxContinuous);
        Radio.SetTxConfig(modem, power, fdev, bandwidth, datarate, coderate, preambleLen,
                          fixLen, crcOn, FreqHopOn, HopPeriod, iqInverted, 1000);
};

/*****************************************************************************/
/* setters/getters                                                           */
/*****************************************************************************/

/// @brief Set maximum number of retries when sending a packet with acknowledgment. Default is 5;
/// @param max_retries
void RadioManagerClass::setMaxRetries(uint16_t max_retries){
    _max_retries = max_retries;
}

/// @brief Return this device's address.
uint8_t RadioManagerClass::getAddress(){
    return _address;
}

/// @brief Return the last send packet id.
uint8_t RadioManagerClass::getLastPacketID(){
    return _last_packet_id;
}


/*****************************************************************************/
/*  helper functions                                                         */
/*****************************************************************************/

/// @brief Reset event structure before assigning new callbacks
void RadioManagerClass::_reset_events() {
    _events.RxError   = NULL;
    _events.TxDone    = NULL;
    _events.RxDone    = NULL;
    _events.RxTimeout = NULL;
    _events.TxTimeout = NULL;
}

/// @brief Store the packet in the private variable ::packet for later use. Transmission quality
/// indicators are also stores
/// @param payload the lora payload
/// @param length the length of the payload
/// @param rssi the transmission RSSI
/// @param snr the transmission SNR
void RadioManagerClass::storePacket(uint8_t *payload, uint16_t length, int16_t rssi, int8_t snr) {
    memcpy((uint8_t*)(&_packet), payload, length);
    _packet.rssi = rssi;
    _packet.snr  = snr;
}


/*****************************************************************************/
/*  recv                                                                     */
/*****************************************************************************/
/// @brief Set the radio in receive mode until a packet is available for the device.
/// @param ack_wait delay in milliseconds betw
/// @return True if a message was received for this device before timeout, else False.
void RadioManagerClass::recv(uint8_t* data, uint8_t* length, uint8_t* from, int16_t* rssi, int16_t* snr) {
    // assign callbacks
    _reset_events();
    _events.RxError = _recv_onRxError;
    _events.RxDone  = _recv_onRxDone;
    _events.TxDone  = _recv_onTxDone;

    state = RadioManagerState::MANAGER_RX;
    while (true) {
        #ifdef DEBUGRADIO
            Serial.print("radio manager state: "); Serial.println(STATEMAP[state]);
            Serial.println(); Serial.flush();
        #endif
        // process interrupt request callbacks
        Radio.IrqProcess();
        switch(state) {
            case RadioManagerState::MANAGER_TX: {
                // wait tiny bit before sending the acknowledgment
                delay(10);

                // send last packet received rssi and snr in the acknowledgment
                uint8_t buff[4] = {highByte(_packet.rssi), lowByte(_packet.rssi),
                                   highByte(_packet.snr),  lowByte(_packet.snr)};

                // make acknowledgment packet and send it
                Packet_t ack = make_packet(_packet.header.src_address,
                                           _address,
                                           ACKNOWLEDGEMENT,
                                           _packet.header.packet_id,
                                           &buff[0],
                                           4);

                Radio.Send((uint8_t*)(&ack), ack.nbytes);
                state = RadioManagerState::LOWPOWER;
                break;
            }
            case RadioManagerState::MANAGER_RX: {
                Radio.Rx(0);
                state = RadioManagerState::LOWPOWER;
                break;
            }
            case RadioManagerState::LOWPOWER: {
                _lowPowerHandler();
                break;
            }
            default: {
                // covers HAS_RECV and HAS_SENT_ACK
                Radio.Sleep();

                // copy content to output
                memcpy(data, _packet.payload, _packet.length);
                *length = _packet.length;
                *from   = _packet.header.src_address;
                *rssi   = _packet.rssi;
                *snr    = _packet.snr;

                // if we received a packet while we are not in this loop
                // we'll get it the next time we call Radio.IrqProcess.
                return;
            }
        }
    }
}

// callbacks for recv

void _recv_onRxDone(uint8_t *data, uint16_t length, int16_t rssi, int8_t snr) {
    // if the packet is addressed at this device, stop receiving
    // it might be better if we keep receiving stuff and put packets
    // in a buffer, especially if this runs on a "gateway" and we might have
    // many nodes that send data. For this usecase, one should use `available`
    #ifdef DEBUGRADIO
        Serial.println("callback _recv_onRxDone fired");
    #endif
    if (get_dest_address(data) == RadioManager.getAddress()){
        // store data into member variable _packet
        RadioManager.storePacket(data, length, rssi, snr);

        // if the packet requires a confirmation send it, othewise just return
        if (get_msg_type(data) == CONFIRMED) {
            RadioManager.state = RadioManagerState::MANAGER_TX;
        } else {
            RadioManager.state = RadioManagerState::HAS_RECV;
        }
    }
}

void _recv_onTxDone() {
    #ifdef DEBUGRADIO
        Serial.println("callback _recv_onTxDone_ack fired");
    #endif
    RadioManager.state = RadioManagerState::HAS_SENT_ACK;
}

void _recv_onRxError() {
    #ifdef DEBUGRADIO
        Serial.println("callback _recv_onRxError fired");
    #endif
    RadioManager.state = RadioManagerState::MANAGER_RX;
}


/*****************************************************************************/
/*  send                                                                     */
/*****************************************************************************/
/// @brief Send the message (with retries) and wait for an acknowledgment
/// Send the message (with retries) and waits for an ack. Returns true
/// if an acknowledgement is received. Synchronous: any message other than the
/// desired ACK received while waiting is discarded. Blocks until an ACK is
/// received or all retries are exhausted (ie up to retries*timeout milliseconds).
/// If the destination address is the broadcast address RH_BROADCAST_ADDRESS (255),
/// the message will be sent as a broadcast, but receiving nodes do not acknowledge,
/// and sendtoWait() returns true immediately without waiting for any acknowledgements.
/// @param payload [in] the payload to be sent
/// @param length [in] the length of the payload, in bytes
/// @param dest_address [in] the address of the destination device
/// @param confirmed [in] true if an acknowledgment is required
/// @param ack_timeout_ms [in] timeout in millisecond for receiving an acknowledgment if confirmed is true
/// @param msg_rssi [OUT] the rssi of the message received by the destination device. This
/// is sent back in the acknowledgment message
/// @param msg_snr [OUT] the snr of the message received by the destination device
/// @param ack_rssi [OUT] the rssi of the acknowledgment message
/// @param ack_snr [OUT] the snr of the acknowledgment message
/// @return
bool RadioManagerClass::send(uint8_t* data,
                              uint8_t length,
                              uint8_t dest_address,
                                 bool confirmed,
                             uint32_t ack_timeout_ms,
                             int16_t* msg_rssi,
                             int16_t* msg_snr,
                             int16_t* ack_rssi,
                             int16_t* ack_snr) {

    // make packet
    Packet_t pckt = make_packet(dest_address,
                                _address,
                                (confirmed == true) ? CONFIRMED : UNCONFIRMED,
                                _last_packet_id,
                                data,
                                length);

    // assign callbacks
    _reset_events();
    _events.TxTimeout = _send_onTxTimeout;
    if (confirmed == true) {
        _events.RxDone    = _send_onRxDone_ack;
        _events.TxDone    = _send_onTxDone_ack;
        _events.RxTimeout = _send_onRxTimeout_ack;
        _events.RxError   = _send_onRxError_ack;
    } else {
        _events.TxDone    = _send_onTxDone;
    }

    state = RadioManagerState::MANAGER_TX;
    uint8_t attempt = 0;

    while (true) {
        #ifdef DEBUGRADIO
            Serial.print("radio manager state: "); Serial.println(STATEMAP[state]);
        #endif
        // process interrupt requests
        Radio.IrqProcess();
        switch(state) {
            case RadioManagerState::MANAGER_TX: {
                attempt += 1;
                Radio.Send((uint8_t*)(&pckt), pckt.nbytes);
                state = LOWPOWER;
                break;
            }
            case RadioManagerState::MANAGER_RX: {
                Radio.Rx(ack_timeout_ms);
                // return after max_retries after we have failed to receive the ACK message
                if (attempt >= _max_retries) {
                    state = HAS_TIMEDOUT;
                } else {
                    state = LOWPOWER;
                }
                break;
            }
            case RadioManagerState::LOWPOWER: {
                _lowPowerHandler();
                break;
            }
            default: {
                // send radio to sleep
                Radio.Sleep();

                // increment packet counter, noting we have at most 3 bits 
                _last_packet_id = (_last_packet_id + 1) % 8;

                // exit point when no acknowledgement is required
                if (state == HAS_SENT || state == HAS_TIMEDOUT) {
                    *msg_rssi = 0;
                    *msg_snr  = 0;
                    *ack_rssi = 0;
                    *ack_snr  = 0;
                    return false;
                }
                
                // exit point when an acknowledgement is required
                if (state == HAS_RECV_ACK) {
                    // store ack packet rssi and snr for output
                    if (msg_rssi != NULL) {*msg_rssi = bytes_to_int16_t(_packet.payload[0], _packet.payload[1]);}
                    if (msg_snr  != NULL) {*msg_snr  = bytes_to_int16_t(_packet.payload[2], _packet.payload[3]);}
                    if (ack_rssi != NULL) {*ack_rssi = _packet.rssi;}
                    if (ack_snr  != NULL) {*ack_snr  = _packet.snr;}
                    return true;
                }
            }
        }
    }
};

// callbacks for send

/// @brief On RadioManagerState::MANAGER_RX done, exit, or keep listening if message is not for us
void _send_onRxDone_ack(uint8_t *data, uint16_t length, int16_t rssi, int8_t snr) {
    #ifdef DEBUGRADIO
        Serial.println("callback _send_onRxDone_ack fired");
    #endif
    // if the message is for us, check it is an acknowledgment
    if (get_dest_address(data) == RadioManager.getAddress()) {
        // store data into member variable _packet
        RadioManager.storePacket(data, length, rssi, snr);

        if (get_msg_type(data) == ACKNOWLEDGEMENT &&
            get_packet_id(data) == RadioManager.getLastPacketID()) {
            RadioManager.state = RadioManagerState::HAS_RECV_ACK;
        }
    } else {
        // otherwise keep listening
        RadioManager.state = RadioManagerState::MANAGER_RX;
    }
}

/// @brief On RadioManagerState::MANAGER_RX error, attempt to send packet again
void _send_onRxError_ack() {
    #ifdef DEBUGRADIO
        Serial.println("callback _send_onRxError_ack fired");
    #endif
    RadioManager.state = RadioManagerState::MANAGER_TX;
}

/// @brief On RadioManagerState::MANAGER_RX timeout, attempt to send packet again
void _send_onRxTimeout_ack() {
    #ifdef DEBUGRADIO
        Serial.println("callback _send_onRxTimeout_ack fired");
    #endif
    RadioManager.state = RadioManagerState::MANAGER_TX;
}

/// @brief On RadiRadioManagerState_t::TX done, with acknowledgment required, wait for the ACK packet
void _send_onTxDone_ack() {
    #ifdef DEBUGRADIO
        Serial.println("callback _send_onTxDone_ack fired");
    #endif
    RadioManager.state = RadioManagerState::MANAGER_RX;
}

/// @brief On RadiRadioManagerState_t::TX done, with no acknowledgment required, exit
void _send_onTxDone() {
    #ifdef DEBUGRADIO
        Serial.println("callback _send_onTxDone fired");
    #endif
    RadioManager.state = RadioManagerState::HAS_SENT;
}

/// @brief On RadiRadioManagerState_t::TX done, with no acknowledgment required, exit
void _send_onTxTimeout() {
    #ifdef DEBUGRADIO
        Serial.println("callback _send_onTxTimeout fired");
    #endif
    RadioManager.state = RadioManagerState::MANAGER_TX;
}