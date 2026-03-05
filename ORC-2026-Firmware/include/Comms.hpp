#ifndef COMMS_HPP
#define COMMS_HPP
#include <stdint.h>
#include <stddef.h>
#include <BluetoothSerial.h>

namespace Comms {
inline BluetoothSerial SerialBT;

// ---------------------------------------------------------------------------
// Packet IDs
// ---------------------------------------------------------------------------
enum PacketID : uint8_t {
    CMD_SET_VELOCITY = 0xA1,
    LOG_TELEMETRY    = 0xB1,
};


// ---------------------------------------------------------------------------
// Packet definitions
// Checksum = XOR of every byte that precedes it in the struct.
// ---------------------------------------------------------------------------

// Host → Robot  (10 bytes)
struct __attribute__((packed)) CmdSetVelocity {
    uint8_t id;         // CMD_SET_VELOCITY
    float   v;          // Linear velocity  (m/s)
    float   w;          // Angular velocity (rad/s)
    uint8_t checksum;
};

// Robot → Host  (14 bytes)
struct __attribute__((packed)) LogTelemetry {
    uint8_t id;         // LOG_TELEMETRY
    float   v;          // Measured linear velocity  (m/s)
    float   w;          // Measured angular velocity (rad/s)
    float   distance;   // Ultrasonic distance       (m)
    uint8_t checksum;
};


// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

inline uint8_t computeChecksum(const uint8_t* data, size_t len) {
    uint8_t cs = 0;
    for (size_t i = 0; i < len; i++) cs ^= data[i];
    return cs;
}

// Returns the expected total packet size for a given ID, or 0 if unknown.
inline size_t packetSize(uint8_t id) {
    switch (id) {
        case CMD_SET_VELOCITY: return sizeof(CmdSetVelocity);
        case LOG_TELEMETRY:    return sizeof(LogTelemetry);
        default:               return 0;
    }
}


// ---------------------------------------------------------------------------
// Main API
// ---------------------------------------------------------------------------

inline void setup(String btName) {
    SerialBT.begin(btName);
}

inline void parse(float& targetV, float& targetW, portMUX_TYPE& mux) {
    static uint8_t buf[32];
    static size_t  bufLen = 0;

    while (SerialBT.available()) {
        uint8_t b = SerialBT.read();

        if (bufLen == 0) {
            // Wait for a recognised header byte
            if (Comms::packetSize(b) == 0) continue;
        }

        buf[bufLen++] = b;
        size_t expected = Comms::packetSize(buf[0]);

        if (bufLen < expected) continue; // need more bytes

        // Full packet received — validate checksum
        uint8_t cs = Comms::computeChecksum(buf, expected - 1);
        if (cs == buf[expected - 1]) {
            switch (buf[0]) {
                case Comms::CMD_SET_VELOCITY: {
                    Comms::CmdSetVelocity pkt;
                    memcpy(&pkt, buf, sizeof(pkt));
                    portENTER_CRITICAL(&mux);
                    targetV = pkt.v;
                    targetW = pkt.w;
                    portEXIT_CRITICAL(&mux);
                    break;
                }
            }
        }
        bufLen = 0; // reset for next packet (discard on bad checksum too)
    }
}

inline void sendTelemetry(float v, float w, float distance) {
    Comms::LogTelemetry log;
    log.id = Comms::LOG_TELEMETRY;
    log.v  = v;
    log.w  = w;
    log.distance = distance;
    log.checksum = Comms::computeChecksum((uint8_t*)&log, sizeof(log) - 1);
    SerialBT.write((uint8_t*)&log, sizeof(log));
}

} // namespace Comms
#endif // COMMS_HPP
