#ifndef MOTOR_HPP
#define MOTOR_HPP
#include <Arduino.h>
#include <ESP32Encoder.h>

// --- MOTOR / ENCODER CONSTANTS ---
constexpr float WHEEL_RADIUS      = 0.032f; // Meters - measure your wheel
constexpr float TICKS_PER_REV     = 840.0f; // DG01D-E: 7 ticks/rev * ~120:1 gearbox - verify by counting

class MotorWithEncoder {
int positivePin;
int negativePin;
int encoderAPin;
int encoderBPin;
int id;
// ESP32Encoder encoder;
int64_t lastCount = 0;
uint32_t lastTime = 0;
float cachedVelocity = 0.0f;

inline static int idCounter = 0;

public:

MotorWithEncoder(int posPin, int negPin, int encAPin, int encBPin) 
    : positivePin(posPin), negativePin(negPin), encoderAPin(encAPin), encoderBPin(encBPin) {
    id = idCounter++;
};

void setup() {
    ledcSetup(2*id, 20000, 8); // Channel 0, 20 kHz, 8-bit resolution
    ledcSetup(2*id+1, 20000, 8); // Channel 1, 20 kHz, 8-bit resolution
    ledcAttachPin(positivePin, 2*id);
    ledcAttachPin(negativePin, 2*id+1);
    // ESP32Encoder::useInternalWeakPullResistors = UP;
    // encoder.attachFullQuad(encoderAPin, encoderBPin);
};

// Poswer = [-1, 1], where negative is reverse and positive is forward
void setPower(float power) {
    if (power > 0) {
        ledcWrite(2*id, 255 * power);
        ledcWrite(2*id+1, 255);
    } else if (power < 0) {
        ledcWrite(2*id, 255);
        ledcWrite(2*id+1, 255 * -power);
    } else {
        ledcWrite(2*id, 255);
        ledcWrite(2*id+1, 255);
    }
};

void stop() {
    ledcWrite(2*id, 0);
    ledcWrite(2*id+1, 0);
};

void setVelocity(float velocity) {
    // Placeholder
    setPower(velocity);
};

float getVelocity() {
    return cachedVelocity;
}

void update() {
    // --- VELOCITY MEASUREMENT ---
    uint32_t now = micros();
    // int64_t  count = encoder.getCount();
    int64_t count = 0;
    uint32_t dt = now - lastTime;
    if (dt > 0) {
        int64_t delta = count - lastCount;
        // (ticks / ticks_per_rev) * circumference / dt_seconds
        cachedVelocity = (delta / TICKS_PER_REV) * (2.0f * PI * WHEEL_RADIUS) / (dt * 1e-6f);
    }
    lastCount = count;
    lastTime  = now;

    // TODO: PI velocity loop
};

}; // MotorWithEncoder

#endif // MOTOR_HPP