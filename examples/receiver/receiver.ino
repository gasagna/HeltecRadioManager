/*

This example demonstrates the functionality for gateways, using the 
Heltec Wireless Stick V3 board. The sketch can be modified for other 
boards with minimal effort. 

In the main loop we use the function recv, which blocks until a 
packet addressed at this device is received. The message information
is then simply displayed on the serial port.

*/
#include "Arduino.h"
#include "HeltecRadioManager.h"

#define FREQUENCY      868000000 // check your region
#define ADDRESS        0         // the address of this gateway device
#define POWER          15        // tx power in dBm

void setup() {
    // init serial port
    Serial.begin(115200); delay(100);

    // init micro controller (necessary for Wireless Tick V3)
    Mcu.begin();

    // set pin mode (for Wireless Tick V3)
    pinMode(LED, OUTPUT);

    // init radio manager
    bool radioinit = RadioManager.begin(ADDRESS, FREQUENCY, POWER);

    if (radioinit == false) {
        Serial.println("unable to start radio");
        while (1) {}
    } else {
        Serial.println("radio started");
    }
}

void loop() {

    // definitions
    uint8_t payload[MAX_PAYLOAD_LEN];
    uint8_t length;
    uint8_t from;
    int16_t rssi;
    int16_t snr;

    // receive data - this is a blocking call
    RadioManager.recv(&(payload[0]), &length, &from, &rssi, &snr);

    // flash LED ON receive (for the Wireless Stick V3 board)
    digitalWrite(LED, HIGH); delay(50); digitalWrite(LED, LOW);

    // once data is available, do what you want with it, e.g. one 
    // could connect to wifi (using the wifi radio available on
    // Heltec's ESP32 LoRa board) and send the data to a remote 
    // database. Here, we just print the packet info, as an example
    Serial.println("Received packet:");
    Serial.printf("  from    : %d\n", from);
    Serial.printf("  rssi    : %d\n", rssi);
    Serial.printf("  snr     : %d\n", snr);
    Serial.printf("  length  : %d\n", length);
    Serial.print("  payload : ");
    for (int i=0; i < length; i++) {
        Serial.printf("%d", payload[i]);
    }
    Serial.println("\n");
}