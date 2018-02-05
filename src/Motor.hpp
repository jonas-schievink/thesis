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

struct MotorConfig {
    /**
     * @brief GPIO for enabling the motor (HIGH=On, LOW=Off).
     */
    int ctrl_pin;
    /**
     * @brief GPIO for changing motor directions (HIGH=Reverse, LOW=Normal).
     */
    int direction_pin;
    /**
     * @brief PWM frequency to request.
     *
     * This is just a *hint* and may not be obeyed. The real frequency will be
     * logged to the console.
     */
    int pwm_freq;
    /**
     * @brief PWM value range to request.
     *
     * This is just a *hint* and may not be obeyed. The real range will be
     * logged to the console.
     */
    int pwm_range;
    /**
     * @brief Abs. limit on the delta time between `Motor::update` calls.
     *
     * This ensures that a big time delta (due to bugs / hiccups elsewhere)
     * doesn't cause us to apply a big PWM signal difference in one step.
     */
    int max_delta_ms;
    /**
     * @brief Max. acceleration in vel%/s.
     */
    float max_accel;
    /**
     * @brief Max. number of direction changes per second.
     */
    float max_dir_changes;

    MotorConfig(int ctrl_pin, int dir_pin);
};

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
    MotorConfig m_config;
    /// @brief Target speed [-1.0, 1.0]
    float m_setpoint;
    /// @brief Actual speed [-1.0, 1.0]
    float m_actual;
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
    Motor(MotorConfig config);

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
