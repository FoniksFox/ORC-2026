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
// ESP32Encoder encoder;
int64_t lastCount = 0;
uint32_t lastTime = 0;
float cachedVelocity = 0.0f;

public:

MotorWithEncoder(int posPin, int negPin, int encAPin, int encBPin) 
    : positivePin(posPin), negativePin(negPin), encoderAPin(encAPin), encoderBPin(encBPin) {};

void setup() {
    pinMode(positivePin, OUTPUT);
    pinMode(negativePin, OUTPUT);
    // ESP32Encoder::useInternalWeakPullResistors = UP;
    // encoder.attachFullQuad(encoderAPin, encoderBPin);
};

// Power = [-1, 1], where negative is reverse and positive is forward
void setPower(float power) {
    if (power > 0) {
        analogWrite(positivePin, (int)(255 * power));
        analogWrite(negativePin, 0);
    } else if (power < 0) {
        analogWrite(positivePin, 0);
        analogWrite(negativePin, (int)(255 * -power));
    } else {
        analogWrite(positivePin, 0);
        analogWrite(negativePin, 0);
    }
};

void stop() {
    analogWrite(positivePin, 0);
    analogWrite(negativePin, 0);
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