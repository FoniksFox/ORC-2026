#include "BluetoothConnection.h"
#include <algorithm>
#include <stdio.h>

BluetoothConnection::BluetoothConnection() {
    // No initialization needed for ESP32 hardware
}

BluetoothConnection::~BluetoothConnection() { 
    disconnect(); 
}

bool BluetoothConnection::isConnected() {
    return SerialBT.hasClient() || SerialBT.connected(); 
}

// --- SERVER LOGIC ---
bool BluetoothConnection::startServer(uint8_t channel, const std::string& esp_name) {
    // Note: The ESP32 Bluetooth Classic library handles the SPP channel automatically.
    return SerialBT.begin(esp_name.c_str()); 
}

bool BluetoothConnection::waitForClient() {
    return SerialBT.hasClient();
}

// --- CLIENT LOGIC ---
bool BluetoothConnection::connect(const std::string& mac_address, uint8_t channel) {
    // Convert MAC string "AA:BB:CC:DD:EE:FF" to uint8_t array
    int b[6];
    sscanf(mac_address.c_str(), "%02x:%02x:%02x:%02x:%02x:%02x", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]);
    uint8_t mac[6] = {(uint8_t)b[0], (uint8_t)b[1], (uint8_t)b[2], (uint8_t)b[3], (uint8_t)b[4], (uint8_t)b[5]};

    SerialBT.begin("ESP32_Client", true); // True means start as Client
    return SerialBT.connect(mac);
}

void BluetoothConnection::disconnect() {
    SerialBT.disconnect();
}

// --- DATA TRANSFER ---
void BluetoothConnection::flushReceiveBuffer() {
    while (SerialBT.available()) SerialBT.read(); // drain hardware buffer
    rx_buffer.clear();
}

bool BluetoothConnection::raw_send(const uint8_t* data, size_t length) {
    return SerialBT.write(data, length) == length;
}

uint8_t BluetoothConnection::calculateChecksum(const uint8_t* data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) checksum ^= data[i];
    return checksum;
}

bool BluetoothConnection::processIncoming(uint8_t* out_packet, size_t struct_size, uint8_t expected_id) {
    if (!isConnected()) return false;

    // Drain the hardware buffer into our vector
    while (SerialBT.available()) {
        rx_buffer.push_back(SerialBT.read());
    }

    // Sliding window parser
    if (rx_buffer.size() >= struct_size) {
        for (size_t i = 0; i <= rx_buffer.size() - struct_size; i++) {
            if (rx_buffer[i] == expected_id) {
                uint8_t expected_cs = calculateChecksum(&rx_buffer[i], struct_size - 1);
                if (expected_cs == rx_buffer[i + struct_size - 1]) {
                    std::copy(rx_buffer.begin() + i, rx_buffer.begin() + i + struct_size, out_packet);
                    rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + i + struct_size);
                    return true;
                }
            }
        }
        if (rx_buffer.size() > 128) rx_buffer.clear(); 
    }
    return false;
}