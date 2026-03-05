#include <Arduino.h>
#include <ESP32Encoder.h>

#include "Pinout.hpp"
#include "Comms.hpp"
#include "Motor.hpp"
#include "DistanceSensor.hpp"

/* TODO */
/**
 * Add moving average to sensors
 * Implement PI velocity loop
 * Implement PID velocity loop
 * Clean up the code in different files
 * Use actual numbers for kinematics and PID constants
 * Define the pins that will actually be used
 * 
 */

// --- ROBOT KINEMATICS ---
const float CENTER_OFFSET = 0.05; // Meters (d) - Dist from axle to geometric center

// --- GLOBALS ---
float targetV = 0, targetW = 0;			// Setpoints (spinlock-protected)
volatile uint32_t startTime = 0;		// Written+read only in ISR, safe as volatile
float distance = 0;						// Written in ISR, read in tasks (spinlock-protected)
constexpr float TRACK_WIDTH = 0.3;		// Meters (T) - Distance between the two wheels (for inverse kinematics)

MotorWithEncoder leftMotor(Pinout::MOTOR_LEFT_POSITIVE, Pinout::MOTOR_LEFT_NEGATIVE, Pinout::ENCODER_LEFT_A, Pinout::ENCODER_LEFT_B);
MotorWithEncoder rightMotor(Pinout::MOTOR_RIGHT_POSITIVE, Pinout::MOTOR_RIGHT_NEGATIVE, Pinout::ENCODER_RIGHT_A, Pinout::ENCODER_RIGHT_B);

DistanceSensor distanceSensor(Pinout::DISTANCE_SENSOR_TRIGGER, Pinout::DISTANCE_SENSOR_ECHO);


static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Task Handles
TaskHandle_t ControlTask;

void controlLoop(void * pvParameters);

void setup() {
    Serial.begin(115200);
    Comms::setup("ORC-2026");

    leftMotor.setup();
    rightMotor.setup();

	distanceSensor.setup();

    // Pin the Control Loop to Core 1 (Priority 1)
    xTaskCreatePinnedToCore(controlLoop, "PID_Task", 4096, NULL, 1, &ControlTask, 1);

	leftMotor.setPower(1.0f);
	rightMotor.setPower(1.0f);
}

void loop() {
    // CORE 0: Trigger HC-SR04 every 60ms
    static uint32_t lastTrigger = 0;
    uint32_t now = millis();
    if (now - lastTrigger >= 60) {
        distanceSensor.trigger();
		lastTrigger = now;
    }

    // CORE 0: Handle Bluetooth incoming packets
    Comms::parse(targetV, targetW, mux);

    vTaskDelay(pdMS_TO_TICKS(20));
}

void controlLoop(void * pvParameters) {
    for(;;) {
        portENTER_CRITICAL(&mux);
        float v = targetV;
        float w = targetW;
        portEXIT_CRITICAL(&mux);

        // --- INVERSE KINEMATICS ---
        // TODO: replace magic 0.5 with (track_width / 2.0f) once measured
        float left_target_vel  = v - (w * TRACK_WIDTH / 2.0f); // m/s
        float right_target_vel = v + (w * TRACK_WIDTH / 2.0f); // m/s

        // TODO: replace 0.0f with actual measured velocities from encoder deltas
		float left_measured_vel = leftMotor.getVelocity();
		float right_measured_vel = rightMotor.getVelocity();
        float measuredV = (left_measured_vel + right_measured_vel) / 2.0f; // m/s
        float measuredW = (right_measured_vel - left_measured_vel) / TRACK_WIDTH; // rad/s

        Comms::sendTelemetry(measuredV, measuredW, distanceSensor.getDistance());

		leftMotor.update();
		rightMotor.update();

        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz
    }
}