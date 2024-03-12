/*
  Copyright 2024 Keith E Henry
  (Based on the ideas and code of many creative individuals before me.)

  MIT License

  Mailbox notifier with ESP-01S.

  Configuration:
    Carefully remove (de-solder) the pullup on EN and add an external 10K pulldown instead.
    Connect RX to EN and connect them to one end of a momentary contact (e.g. reed) switch.
    Connect the other end of the switch to Vcc.
    Connect Vcc and Gnd to 3.3V power.

    Optionally, add a jumper between GPIO0 and GPIO2.

  Operation:
    When the switch is activated, EN high enables the ESP8266. [Reset is pulled high on the module.]
    RX is immediately driven high, setting EN high, allowing the switch to be deactivated.
    When finish sending the MAC address to the receiver, RX is released, allowing the 10K pulldown
        to disable the ESP8266. [RX never pulls low, in case the switch remains activated.]
    If the switch does remain activated, deep sleep mode is set, until EN goes low.
    Disabled mode is spec'd to use only 3uA, as opposed to 20uA for deep sleep.

    Both GPIO0 and GPIO2 are pulled high, as needed for normal power up, on the module.
    Once started, GPIO0 is driven low and GPIO2 is sensed. If GPIO2 is now low, there must
      be a jumper between them. If so, forget about normal operation and use the ElegantOTA
      library to reload new programming.
*/
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <ElegantOTA.h>
#include <WiFiClient.h>
#include <WifiAndServerSettings.h>
#include <espnow.h>
/* of the form:
const char *ssid = ".....";
const char *password = ".....";
// Receiver MAC address:
uint8_t broadcastAddress[] = {0x60, 0x50, 0x40, 0x30, 0x20, 0x10};
*/

ESP8266WebServer server(80);

#define DEBUG false  // Set to true for debug output, false for no debug output
#define DEBUG_SERIAL \
    if (DEBUG) Serial

// Pins accessible on the ESP-01S.
// Also: Vcc, Gnd, En, Rst
static const uint8_t D3 = 0;  // GPIO 0
static const uint8_t D4 = 2;  // GPIO 2
static const uint8_t RX = 3;  // GPIO 3
static const uint8_t TX = 1;  // GPIO 1

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
    DEBUG_SERIAL.print("Last Packet Send Status: ");
    if (sendStatus == 0) {
        DEBUG_SERIAL.println("Delivery success");
    } else {
        DEBUG_SERIAL.println("Delivery fail");
    }

    // Release the output driver on RX.
    // The external pulldown on CH_EN will power down the chip.
    pinMode(RX, INPUT);

    // Just in case the switch is still on.
    ESP.deepSleep(0);
}

bool modeESP;

void setup() {
    // RX is connected to CH_EN which has temporarily been pulled high.
    //  Drive RX HIGH to continue to enable the ESP-01S.
    digitalWrite(RX, HIGH);
    pinMode(RX, OUTPUT);

    // Test if D3 an D4 are jumper'd. Both have 12K pullup resistors
    //  on the module and are high at boot.
    // If so, run OTA update instead of the sending via ESP-Now.
    pinMode(D4, INPUT);
    digitalWrite(D3, LOW);
    pinMode(D3, OUTPUT);
    modeESP = digitalRead(D4);

    if (modeESP) {
        //**** Send an ESP-Now message. ****

        // Init DEBUG_SERIAL Monitor
        DEBUG_SERIAL.begin(115200);
        if (DEBUG) delay(500);

        // Set device as a Wi-Fi Station
        WiFi.mode(WIFI_STA);

        // Init ESP-NOW
        if (esp_now_init() != 0) {
            DEBUG_SERIAL.println("Error initializing ESP-NOW");
            return;
        }

        // Once ESPNow is successfully Init, we will register for Send CB to
        // get the status of Transmitted packet
        esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
        esp_now_register_send_cb(OnDataSent);

        // Register peer
        esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

        // Allocate one big C-string
        char myData[4];
        // sprintf(myData, "{\"I\": %d}", analogRead(0));
        sprintf(myData, "{ }");

        // Send message via ESP-NOW
        esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

    } else {

        //**** OTA update. ****

        // Handle server and ElegantOTA:
        DEBUG_SERIAL.begin(115200);

        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid, password);
        DEBUG_SERIAL.println("");

        // Wait for connection
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            DEBUG_SERIAL.print(".");
        }
        DEBUG_SERIAL.println("");
        DEBUG_SERIAL.print("Connected to ");
        DEBUG_SERIAL.println(ssid);
        DEBUG_SERIAL.print("IP address: ");
        DEBUG_SERIAL.println(WiFi.localIP());

        server.on("/", []() {
            server.send(200, "text/plain", "Hi! This is ElegantOTA Demo.");
        });

        ElegantOTA.begin(&server);  // Start ElegantOTA
        server.begin();
        DEBUG_SERIAL.println("HTTP server started");

        // My setup code here:
        // Initialize LED digital pin as an output.
        digitalWrite(LED_BUILTIN, HIGH);  // HIGH is off
        pinMode(LED_BUILTIN, OUTPUT);
    }
}

void loop(void) {
    //**** Only if in OTA update mode. ****
    if (!modeESP) {
        // Handle server and ElegantOTA:
        server.handleClient();
        ElegantOTA.loop();

        // My loop here:
        // wait (ms)
        delay(2000);

        for (int i = 0; i < 5; i++) {
            // turn the LED ON and wait
            digitalWrite(LED_BUILTIN, LOW);
            delay(500);

            // turn the LED OFF and wait
            digitalWrite(LED_BUILTIN, HIGH);
            delay(500);
        }
    }
}