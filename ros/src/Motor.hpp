/**
 * @file Motor.hpp
 * @brief Provides the `Motor` class to control speed and direction of DC motors
 */

#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "PiGPIO.hpp"

/*

Future work:
- Braking (should be possible by setting speed to 0 and reversing direction)
- "Actual" speed integration (wheel m/s instead of 0% - 100%)

*/

/**
 * @brief Controls speed and direction of a single motor using GPIOs.
 *
 * This class does *not* perform PID speed control, but it will prevent
 * "dangerous" operations such as rapidly changing speed or direction.
 */
class Motor {
    pigpio::Pin m_speed_pin;
    pigpio::Pin m_dir_pin;
    /// Target speed [-1.0, 1.0]
    float m_setpoint;

    /// @brief Set motor speed without safety checks.
    void set_direct(float speed);

public:
    /**
     * @brief Create a Motor object using speed and direction pins
     * @param speed_pin @ref Pin object controlling the motor speed via PWM
     * @param dir_pin @ref Pin object controlling the direction of the motor
     *
     * @ref speed_pin is assumed to turn on the motor when high (3.3V),
     * @ref dir_pin is assumed to make the motor turn backwards when high.
     */
    Motor(pigpio::Pin speed_pin, pigpio::Pin dir_pin);

    /**
     * @brief Sets the target speed and direction of the motor
     * @param speed Speed of the motor as a float in range [-1.0, 1.0]
     *
     * If @ref speed parameter is negative, the motor will move backwards.
     * Otherwise, it will move forwards.
     */
    void set(float speed);

    /**
     * @brief Must be called in regular intervals to update the motor speed
     */
    void update();
};

#endif  // MOTOR_HPP
