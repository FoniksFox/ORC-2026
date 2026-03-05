#ifndef PINOUT_HPP
#define PINOUT_HPP

namespace Pinout {
    // Motor Driver (PWM)
    constexpr int MOTOR_LEFT_POSITIVE = 32;
    constexpr int MOTOR_LEFT_NEGATIVE = 33;
    constexpr int MOTOR_RIGHT_POSITIVE = 25;
    constexpr int MOTOR_RIGHT_NEGATIVE = 26;

    // Motor Encoder (PCNT)
    constexpr int ENCODER_LEFT_A = 34;
    constexpr int ENCODER_LEFT_B = 35;
    constexpr int ENCODER_RIGHT_A = 32;
    constexpr int ENCODER_RIGHT_B = 33;

    // Distance Sensor
    constexpr int DISTANCE_SENSOR_TRIGGER = 22;
    constexpr int DISTANCE_SENSOR_ECHO = 23;
}

#endif // PINOUT_HPP