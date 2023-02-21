/*

This example demonstrates the functionality for node, using the 
Heltec CubeCell Plus board as an example. The sketch can be modified 
for other boards with minimal effort. Please refer to the example
in the CubeCell-Arduino repository to understand how to use the 
neopixel, the oled display and how to put the device to sleep when
not in use.

In the main loop we use the function send, to send a message to the
gateway at address 0. The function blocks until an acknowledgment is
sent back from the gateway device. We then sleep for 5 seconds and repeat.

*/

#include "Arduino.h"
// #include <Wire.h>
#include "HeltecRadioManager.h"
#include "CubeCell_NeoPixel.h"
#include "HT_SH1107Wire.h"

#define FREQUENCY      868000000
#define ADDRESS        1
#define GATEWAYADDRESS 0
#define POWER          15
#define MAX_RETRIES    3
#define ACK_ENABLED    true
#define ACK_TIMEOUT_MS 1000
#define PAYLOAD_LEN    10
#define SLEEPDELAY     5000

// neopixel
extern CubeCell_NeoPixel pixels;

// display
extern SH1107Wire display;

int16_t n_ack_recvd    = 0;                   // number of acknowledgments received
int16_t n_packets_sent = 0;                   // number of packets sent
char last_packet_status[9];                   // last packet status
char display_buffer[100];                     // display buff
uint8_t payload[PAYLOAD_LEN] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};  // dummy payload
int16_t msg_rssi, msg_snr, ack_rssi, ack_snr; // store rssi and srn of ack message here

// flash neopixel
void setRGB(uint8_t r, uint8_t g, uint8_t b, uint32_t _delay = 50) {
    pixels.setPixelColor(0, pixels.Color(r, g, b));
    pixels.show();
    delay(_delay);
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show();
}

/// low power stuff
bool sleepTimerExpired;

// call back on time end
static void wakeUp() {
    sleepTimerExpired = true;
}

static void lowPowerSleep(uint32_t sleeptime) {
    TimerEvent_t sleepTimer;
    sleepTimerExpired = false;
    TimerInit(     &sleepTimer, &wakeUp );
    TimerSetValue( &sleepTimer, sleeptime );
    TimerStart(    &sleepTimer );
    while (!sleepTimerExpired) lowPowerHandler();
    TimerStop( &sleepTimer );
}

void setup() {
    // init serial port
    Serial.begin(115200); delay(100);

    // begin pixels
    pixels.begin();

    // begin display
    display.init();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);

    // init radio manager
    RadioManager.begin(ADDRESS, FREQUENCY, POWER);
    RadioManager.setMaxRetries(MAX_RETRIES);
}

void loop() {
    // send the data to address 0 and wait for confirmation
    bool has_recv_ack = RadioManager.send(&payload[0], PAYLOAD_LEN, GATEWAYADDRESS,
                                          ACK_ENABLED, ACK_TIMEOUT_MS, &msg_rssi,
                                          &msg_snr, &ack_rssi, &ack_snr);
    n_packets_sent += 1;

    if (has_recv_ack) {
        setRGB(255, 0, 0, 100);
        sprintf(last_packet_status, "received");
        n_ack_recvd += 1;
    } else {
        setRGB(0, 0, 255, 100);
        sprintf(last_packet_status, "missed  ");
    }

    float rx_rate = static_cast<float>(n_ack_recvd) / n_packets_sent;
    sprintf(display_buffer, "last packet: %s\npckt   gtway    node\n%d:      %d/%d   %d/%d\nackd/sent: %d/%d\nrxrate: %.5f",
            last_packet_status, RadioManager.getLastPacketID(), msg_rssi, msg_snr, ack_rssi, ack_snr, n_ack_recvd, n_packets_sent, rx_rate);

    display.clear();
    display.resetDisplay();
    display.drawString(0, 0, display_buffer);
    display.display();

    // can also send to serial port for monitoring, if desired
    // Serial.println(display_buffer);

    // wait
    lowPowerSleep(SLEEPDELAY);
}