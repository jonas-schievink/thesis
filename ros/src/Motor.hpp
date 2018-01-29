/**
 * @file Motor.hpp
 * @brief Provides the `Motor` class to control speed and direction of DC motors
 */

#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "PiGPIO.hpp"
#include <chrono>

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
    typedef std::chrono::time_point<std::chrono::high_resolution_clock> UpdateTime;

    pigpio::Pin m_speed_pin;
    pigpio::Pin m_dir_pin;
    /// @brief Target speed [-1.0, 1.0]
    float m_setpoint;
    /// @brief Actual speed [-1.0, 1.0]
    float m_actual;
    /// @brief Max. acceleration (change in speed per second).
    float m_maxAccel;
    /// @brief Delay in seconds between direction changes.
    float m_dirChangeDelay;
    /// @brief Time of the last update or construction.
    UpdateTime m_lastUpdate;
    int m_pwmRange;

    /// @brief Set motor speed without safety checks.
    void set_direct(float speed);

    /**
     * @brief Attempt to set motor direction to @ref dir
     * @return `true` if successful, `false` if the limit was hit
     */
    bool set_direction(bool dir);

public:
    /**
     * @brief Create a Motor object using speed and direction pins
     * @param speed_pin @ref Pin object controlling the motor speed via PWM
     * @param dir_pin @ref Pin object controlling the direction of the motor
     *
     * @ref speed_pin is assumed to turn on the motor when high (3.3V),
     * @ref dir_pin is assumed to make the motor turn backwards when high.
     * Safe default values for maximum acceleration and direction changes will
     * be used.
     */
    Motor(pigpio::Pin speed_pin, pigpio::Pin dir_pin);

    /**
     * @brief Sets the maximum acceleration in "percent/s".
     *
     * Note that the total range of motor speeds is, at this point, [-1.0, 1.0].
     * Therefore, setting an acceleration of 1.0 will allow a complete reversal
     * (from highest speed in one direction to highest speed in the other
     * direction) in 2 seconds.
     */
    void setMaxAcceleration(float maxAccel);

    /**
     * @brief Sets the maximum number of direction changes per second.
     *
     * It's best to keep this pretty low, because every direction change
     * effectively shorts out the H-Bridge for a very short time. Doing this
     * frequently can even be audible and will cause the H-Bridge to get quite
     * warm.
     */
    void setMaxDirChanges(unsigned int changesPerSecond);

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

    /**
     * @brief Whether the motor has reached its set point.
     *
     * Note that this only concerns the software protection features. It does
     * not guarantee that the motor actually runs at some defined speed.
     */
    bool reachedSetPoint() const;
};

#endif  // MOTOR_HPP
