/*

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

// Receiver MAC Address
// D1 mimi V4:
// uint8_t broadcastAddress[] = {0x60, 0x01, 0x94, 0x45, 0x75, 0xb8};
// ESP-01S:
uint8_t broadcastAddress[] = {0x60, 0x01, 0x94, 0x25, 0x11, 0xfb};

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