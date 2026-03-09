#include <Arduino.h>
#include <ESP32Encoder.h>

#include "Pinout.hpp"
#include "Comms.hpp"
#include "Motor.hpp"
#include "DistanceSensor.hpp"
#include "BluetoothConnection.h"

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

//BLUETOOTH CODE START
BluetoothConnection bt;
unsigned long lastSendTime = 0;
float targetRightV = 0.0f, targetLeftV = 0.0f;
const int UPDATE_TIME = 5000; //refresh time for each update
//BLUETOOTH CODE END

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Task Handles
TaskHandle_t ControlTask;

void controlLoop(void * pvParameters);

void setup() {
    leftMotor.setup();
    rightMotor.setup();
	distanceSensor.setup();
    
    leftMotor.setPower(1.0f);
	rightMotor.setPower(1.0f);

    //BLUETOOTH CODE START
    Serial.begin(115200);
    Serial.println("Starting ESP32 Bluetooth Server...");
    
    // The PC will look for this exact name
    if (bt.startServer(4, "ESP32_Robot")) {
        Serial.println("Server active. Waiting for PC to connect...");
    } else {
        Serial.println("Failed to start Bluetooth!");
    }
    //BLUETOOTH CODE END

    // Pin the Control Loop to Core 1 (Priority 1)
    // xTaskCreatePinnedToCore(controlLoop, "PID_Task", 4096, NULL, 1, &ControlTask, 1);
}

void loop() {
    // CORE 0: Trigger HC-SR04 every 60ms
    static uint32_t lastTrigger = 0;
    uint32_t now = millis();
    if (now - lastTrigger >= 60) {
        distanceSensor.trigger();
		lastTrigger = now;
    }

    //BLUETOOTH CODE START
    if (!bt.isConnected()) {
        delay(100);
        return;
    }

    UpdateVelocity vel;
    SetMode mode;
    if (bt.receive(vel, UPDATE_VELOCITY_ID)) {
        Serial.println("\n--- Received Update Command ---");
        Serial.print("Right Velocity : "); Serial.println(vel.rightVelocity);
        Serial.print("Left Velocity: "); Serial.println(vel.leftVelocity);
        targetRightV = vel.rightVelocity;
        targetLeftV = vel.leftVelocity;

        //Reply to config changes
        LogTelemetry telemetry = {LOG_TELEMETRY_ID, targetRightV, targetLeftV, distance, 0};
        bt.send(telemetry);
        lastSendTime = millis();
    } else if (bt.receive(mode, SET_MODE_ID)) {
        Serial.println("\n--- Received Set Mode Command ---");
        Serial.print("New Mode : "); Serial.println(mode.mode);
        if (mode.mode == 0) { // STOP MODE
            targetLeftV = 0.0f;
            targetRightV = 0.0f;
        } else if (mode.mode == 1) { // auto mode (para Linea)

        } else { // tank mode

        }
        // USA ESTO PARA ENVIAR EL UPDATE AL CLIENTE
        // LogTelemetry telemetry = {LOG_TELEMETRY_ID, targetRightV, targetLeftV, distance, 0};
        // bt.send(telemetry);
        // lastSendTime = millis();

    }

    if (millis() - lastSendTime >= UPDATE_TIME) {
        LogTelemetry response = {LOG_TELEMETRY_ID, targetRightV, targetLeftV, distance, 0};
        if (bt.send(response)) {
            Serial.println("Sent telemetry update");
        }

    }
    //BLUETOOTH CODE END


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