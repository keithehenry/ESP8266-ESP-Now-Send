/*

*/
#include <ESP8266WiFi.h>
#include <espnow.h>

//#define RX 3          // GPIO 3 is RX, no pullup, safe at boot

#define DEBUG false   // Set to true for debug output, false for no debug output
#define DEBUG_SERIAL \
    if (DEBUG) Serial

// Receiver MAC Address
// D1 mimi V4:
// uint8_t broadcastAddress[] = {0x60, 0x01, 0x94, 0x45, 0x75, 0xb8};
// ESP-01S:
uint8_t broadcastAddress[] = {0x60, 0x01, 0x94, 0x25, 0x11, 0xfb};

// Just one big string
typedef struct struct_message {
    char a[250];
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
    DEBUG_SERIAL.print("Last Packet Send Status: ");
    if (sendStatus == 0) {
        DEBUG_SERIAL.println("Delivery success");
    } else {
        DEBUG_SERIAL.println("Delivery fail");
    }
}

void setup() {
    // Pull the P-channel MosFET gate low immediately to source power
    digitalWrite(RX, LOW);

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

    // Set values to send
    randomSeed(analogRead(0));
    sprintf(myData.a, "{\"D\": %ld}", random(10000));

    // Send message via ESP-NOW
    esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
}

void loop() {}