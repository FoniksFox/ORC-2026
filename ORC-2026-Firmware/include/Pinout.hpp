#ifndef PINOUT_HPP
#define PINOUT_HPP

namespace Pinout {
    // Motor Driver (PWM)
    constexpr int MOTOR_LEFT_POSITIVE = 15;
    constexpr int MOTOR_LEFT_NEGATIVE = 02;
    constexpr int MOTOR_RIGHT_POSITIVE = 0;
    constexpr int MOTOR_RIGHT_NEGATIVE = 4;

    // Motor Encoder (PCNT) (not used)
    constexpr int ENCODER_LEFT_A = 34;
    constexpr int ENCODER_LEFT_B = 35;
    constexpr int ENCODER_RIGHT_A = 32;
    constexpr int ENCODER_RIGHT_B = 33;

    // Distance Sensor
    constexpr int DISTANCE_SENSOR_TRIGGER = 12;
    constexpr int DISTANCE_SENSOR_ECHO = 14;
}

#endif // PINOUT_HPP