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
const int UPDATE_TIME = 100; //refresh time for each update
int curr_mode = 0;

void setup() {
    // ESP32Encoder::useInternalWeakPullResistors = UP;
    leftMotor.setup();
    rightMotor.setup();
	distanceSensor.setup();

    //BLUETOOTH CODE START
    Serial.begin(9600);
    Serial.println("Starting ESP32 Bluetooth Server...");
    
    // The PC will look for this exact name
    if (bt.startServer(1, "ESP32_Robot")) {
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
        
        bt.rx_buffer.clear();
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
        lastSendTime = millis();
    }

    if (curr_mode == 0) { // STOP MODE
        targetLeftV = 0.0f;
        targetRightV = 0.0f;
    } else if (curr_mode == 1) { // competition line mode
        constexpr float    EMERGENCY_DIST =  5.0f;  // cm: wall → emergency stop
        constexpr float    TOGGLE_FAR     = 15.0f;  // cm:  5–15cm zone → hold to start/stop
        constexpr float    LEFT_FAR       = 25.0f;  // cm: 15–25cm zone → timed left turn
        constexpr float    RIGHT_FAR      = 40.0f;  // cm: 25–40cm zone → timed right turn
        constexpr uint32_t TOGGLE_HOLD_MS = 600;    // ms: how long to hold in toggle zone
        constexpr uint32_t LEFT_TURN_MS   = 400;    // ms: left turn duration  (~45°, tune this)
        constexpr uint32_t RIGHT_TURN_MS  = 400;    // ms: right turn duration (~45°, tune this)
        constexpr float    TURN_SPEED     = 0.8f;   // pivot speed (0–1)

        static bool     running      = false; // robot starts stopped
        static bool     inTurn       = false;
        static int      turnDir      = 0;
        static uint32_t turnStart    = 0;
        static bool     toggleArmed  = true;  // requires hand removal before re-triggering
        static bool     inToggleZone = false;
        static uint32_t toggleStart  = 0;

        float    distance = distanceSensor.getDistance();
        uint32_t nowMs    = millis();

        if (distance > 0.0f && distance < EMERGENCY_DIST) {
            // Emergency: stop and reset
            running = false;
            inTurn  = false;
            targetLeftV  = 0.0f;
            targetRightV = 0.0f;
        } else if (inTurn) {
            // BLIND TURN: sensor ignored, execute timed turn
            uint32_t dur = (turnDir == -1) ? LEFT_TURN_MS : RIGHT_TURN_MS;
            if (nowMs - turnStart >= dur) {
                inTurn = false;
                targetLeftV  = 1.0f;
                targetRightV = 1.0f;
            } else if (turnDir == -1) { // LEFT
                targetLeftV  = -TURN_SPEED;
                targetRightV =  TURN_SPEED;
            } else {                    // RIGHT
                targetLeftV  =  TURN_SPEED;
                targetRightV = -TURN_SPEED;
            }
        } else {
            // SENSOR ACTIVE: check toggle zone first
            bool handInToggle = (distance > 0.0f && distance < TOGGLE_FAR);
            if (handInToggle && toggleArmed) {
                if (!inToggleZone) {
                    inToggleZone = true;
                    toggleStart  = nowMs;
                } else if (nowMs - toggleStart >= TOGGLE_HOLD_MS) {
                    running      = !running;
                    toggleArmed  = false;   // disarm until hand is removed
                    inToggleZone = false;
                }
            } else if (!handInToggle) {
                inToggleZone = false;
                toggleArmed  = true;        // hand gone → rearm toggle
            }

            if (!running) {
                targetLeftV  = 0.0f;
                targetRightV = 0.0f;
            } else if (distance >= TOGGLE_FAR && distance < LEFT_FAR) {
                inTurn    = true;
                turnDir   = -1;
                turnStart = nowMs;
            } else if (distance >= LEFT_FAR && distance < RIGHT_FAR) {
                inTurn    = true;
                turnDir   = 1;
                turnStart = nowMs;
            } else {
                // no hand (> 40cm) or sensor timeout → go straight
                targetLeftV  = 1.0f;
                targetRightV = 1.0f;
            }
        }
    } else {
        // tank mode
    }


    rightMotor.setVelocity(targetRightV);
    leftMotor.setVelocity(targetLeftV);


    vTaskDelay(pdMS_TO_TICKS(20));
}