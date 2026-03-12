#pragma once
#include <stdint.h>
#include <string>
#include <vector>
#include "BluetoothSerial.h"

// --- Packet IDs ---
#define UPDATE_VELOCITY_ID 0x01
#define LOG_TELEMETRY_ID 0x02
#define SET_MODE_ID 0x03

// --- Structs ---
#pragma pack(push, 1)
struct UpdateVelocity {
    uint8_t id;
    float   rightVelocity;
    float   leftVelocity;
    uint8_t checksum;
};

struct LogTelemetry {
    uint8_t id;
    float   rightVelocity;
    float   leftVelocity;
    float   distance;
    uint8_t checksum;
};

struct SetMode { // 0: stop, 1: auto, 2: tank
    uint8_t id;
    int32_t mode;
    uint8_t checksum;
};

#pragma pack(pop)

class BluetoothConnection {
public:
    BluetoothConnection();
    ~BluetoothConnection();

    std::vector<uint8_t> rx_buffer;
    // Server mode
    bool startServer(uint8_t channel = 4, const std::string& esp_name = "ESP32_Robot");
    bool waitForClient();

    // Client mode
    bool connect(const std::string& mac_address, uint8_t channel = 4);
    
    void disconnect();
    bool isConnected();
    void flushReceiveBuffer();

    template <typename T>
    bool send(T& packet) {
        if (!isConnected()) return false;
        uint8_t* data = reinterpret_cast<uint8_t*>(&packet);
        packet.checksum = calculateChecksum(data, sizeof(T) - 1);
        return raw_send(data, sizeof(T));
    }

    template <typename T>
    bool receive(T& packet, uint8_t expected_id) {
        return processIncoming(reinterpret_cast<uint8_t*>(&packet), sizeof(T), expected_id);
    }

private:
    BluetoothSerial SerialBT;

    bool raw_send(const uint8_t* data, size_t length);
    bool processIncoming(uint8_t* out_packet, size_t struct_size, uint8_t expected_id);
    uint8_t calculateChecksum(const uint8_t* data, size_t length);
};