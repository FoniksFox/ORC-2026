#ifndef DISTANCESENSOR_HPP
#define DISTANCESENSOR_HPP
#include <Arduino.h>

class DistanceSensor {

    int trigPin;
    int echoPin;
    volatile uint32_t startTime = 0;
    float cachedDistance = 0.0f;  // Written in ISR, read from task (mux-protected)
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

    // --- Static instance registry (factory) ---
    static constexpr int MAX_SENSORS = 2;
    inline static DistanceSensor* instances[MAX_SENSORS] = {};
    inline static int instanceCount = 0;

    // One ISR stub per slot — each calls handleISR() on its instance
    static void IRAM_ATTR isr0() { instances[0]->handleISR(); }
    static void IRAM_ATTR isr1() { instances[1]->handleISR(); }

    void IRAM_ATTR handleISR() {
        if (digitalRead(echoPin) == HIGH) {
            startTime = micros();
        } else {
            uint32_t duration = micros() - startTime;
            portENTER_CRITICAL_ISR(&mux);
            cachedDistance = duration * 0.000017f; // microseconds → meters
            portEXIT_CRITICAL_ISR(&mux);
        }
    }

public:
    DistanceSensor(int trigPin, int echoPin) : trigPin(trigPin), echoPin(echoPin) {}

    void setup() {
        pinMode(trigPin, OUTPUT);
        pinMode(echoPin, INPUT);

        static void (*stubs[MAX_SENSORS])() = { isr0, isr1 };
        int id = instanceCount++;
        instances[id] = this;
        attachInterrupt(digitalPinToInterrupt(echoPin), stubs[id], CHANGE);
    }

    // Call periodically (e.g. every 60ms from loop()) to start a measurement
    void trigger() {
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
    }

    // Returns the last completed measurement in meters
    float getDistance() {
        portENTER_CRITICAL(&mux);
        float d = cachedDistance;
        portEXIT_CRITICAL(&mux);
        return d;
    }

}; // DistanceSensor

#endif // DISTANCESENSOR_HPP