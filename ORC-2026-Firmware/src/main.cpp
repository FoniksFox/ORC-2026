#include <Arduino.h>
#include <ESP32Encoder.h>

#include "Pinout.hpp"
#include "Comms.hpp"
#include "Motor.hpp"
#include "DistanceSensor.hpp"
#include "BluetoothConnection.h"

// --- GLOBALS ---
float targetV = 0, targetW = 0;
volatile uint32_t startTime = 0;
constexpr float TRACK_WIDTH = 0.3;

MotorWithEncoder leftMotor(Pinout::MOTOR_LEFT_POSITIVE, Pinout::MOTOR_LEFT_NEGATIVE, Pinout::ENCODER_LEFT_A, Pinout::ENCODER_LEFT_B);
MotorWithEncoder rightMotor(Pinout::MOTOR_RIGHT_POSITIVE, Pinout::MOTOR_RIGHT_NEGATIVE, Pinout::ENCODER_RIGHT_A, Pinout::ENCODER_RIGHT_B);

DistanceSensor distanceSensor(Pinout::DISTANCE_SENSOR_TRIGGER, Pinout::DISTANCE_SENSOR_ECHO);

BluetoothConnection bt;
unsigned long lastSendTime = 0;
float targetRightV = 0.0f, targetLeftV = 0.0f;
const int UPDATE_TIME = 5000; //refresh time for each update
int curr_mode = 0;

void setup() {
    leftMotor.setup();
    rightMotor.setup();
	distanceSensor.setup();

    //BLUETOOTH CODE START
    Serial.begin(9600);
    Serial.println("Starting ESP32 Bluetooth Server...");
    
    // The PC will look for this exact name
    if (bt.startServer(4, "ESP32_Robot")) {
        // Serial.println("Server active. Waiting for PC to connect...");
    } else {
        // Serial.println("Failed to start Bluetooth!");
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
        // Serial.println("Distance: " + String(distanceSensor.getDistance()) + " cm");
    };

    // //BLUETOOTH CODE START
    // if (!bt.isConnected()) {
    //     delay(100);
    //     return;
    // }

    UpdateVelocity vel;
    SetMode mode;
    if (bt.receive(vel, UPDATE_VELOCITY_ID)) {
        // Serial.println("\n--- Received Update Command ---");
        // Serial.print("Right Velocity : "); Serial.println(vel.rightVelocity);
        // Serial.print("Left Velocity: "); Serial.println(vel.leftVelocity);
        targetRightV = vel.rightVelocity;
        targetLeftV = vel.leftVelocity;

        //Reply to config changes
        LogTelemetry telemetry = {LOG_TELEMETRY_ID, targetRightV, targetLeftV, distanceSensor.getDistance(), 0};
        bt.send(telemetry);
        lastSendTime = millis();
    } else if (bt.receive(mode, SET_MODE_ID)) {
        // Serial.println("\n--- Received Set Mode Command ---");
        // Serial.print("New Mode : "); Serial.println(mode.mode);
        
        curr_mode = mode.mode;

        // USA ESTO PARA ENVIAR EL UPDATE AL CLIENTE
        // LogTelemetry telemetry = {LOG_TELEMETRY_ID, targetRightV, targetLeftV, distanceSensor.getDistance(), 0};
        // bt.send(telemetry);
        // lastSendTime = millis();

    }

    if (millis() - lastSendTime >= UPDATE_TIME) {
        LogTelemetry response = {LOG_TELEMETRY_ID, targetRightV, targetLeftV, distanceSensor.getDistance(), 0};
        if (bt.send(response)) {
            // Serial.println("Sent telemetry update");
        }

    }

    if (curr_mode == 0) { // STOP MODE
        targetLeftV = 0.0f;
        targetRightV = 0.0f;
    } else if (curr_mode == 1) { // auto mode (para Linea)
        auto distance = distanceSensor.getDistance();
        if (distance < 10.0f) {
            targetRightV = 0.0f;
            targetLeftV = 0.0f;
        } else if (distance < 30.0f) { // Steer right
            targetRightV = 1.0f;
            targetLeftV = (distance - 20.0f) / 10.0f; // Linear scaling from 0 to 1 as distance goes from 30 to 10
        } else if (distance < 40.0f) { // Go stright
            targetLeftV = 1.0f;
            targetRightV = 1.0f;
        } else if (distance < 60) { // Steer left
            targetLeftV = 1.0f;
            targetRightV = (distance - 50.0f) / 10.0f; // Linear scaling from 0 to 1 as distance goes from 40 to 60
        } else { // Stop
            targetLeftV = 0.0f;
            targetRightV = 0.0f;
        }
    } else { // tank mode

    }

    rightMotor.setVelocity(targetRightV);
    leftMotor.setVelocity(targetLeftV);


    vTaskDelay(pdMS_TO_TICKS(20));
}