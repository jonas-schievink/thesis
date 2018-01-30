/**
 * @file PiGPIO.hpp
 * @brief C++ wrapper around pigpio
 */

#ifndef PIGPIO_HPP
#define PIGPIO_HPP

#include <stdexcept>
#include <string>

#include <pigpio.h>

namespace pigpio {

/**
 * @brief The exception class used by the pigpio wrapper.
 */
class GPIOException : public std::runtime_error
{
    int m_code;

public:
    GPIOException(int code);
};

/**
 * @brief Pull-up/-down resistor configuration
 */
enum class PullUpDown {
    /**
     * @brief Enable the internal pull-up resistor
     */
    PullUp,
    /**
     * @brief Enable the internal pull-down resistor
     */
    PullDown,
    /**
     * @brief Enable no pull-up or pull-down resistor
     */
    None,
};

/**
 * @brief A GPIO pin.
 *
 * This class encapsulates a single user-controllable pin on the Raspberry Pi's
 * expansion header and allows configuring, controlling and reading it.
 */
class Pin {
    int m_num;

public:
    /**
     * @brief Create a new pin object acting on the given GPIOn pin.
     * @param number The pin number, as specified in the broadcom datasheet.
     */
    Pin(int number);

    /**
     * @brief Get the GPIO number this `Pin` instance was created from.
     */
    int number() const;

    /**
     * @brief Configures this pin as an output
     */
    void output();

    /**
     * @brief Configures this pin as an input
     */
    void input();

    /**
     * @brief Set pull-up/-down configuration
     */
    void pullupdown(PullUpDown config);

    /**
     * @brief Reads the logic level of this GPIO
     * @returns `false` for low level, `true` for high level
     */
    bool digitalRead() const;

    /**
     * @brief Sets the output level of this GPIO to a high or low level (3.3V / 0V).
     * @param level true for high logic level, false for a low level
     */
    void digitalWrite(bool level);

    /**
     * @brief Starts a PWM signal on this pin
     * @param duty The dutycycle (by default between 0 and 255)
     */
    void pwm(unsigned int duty);

    /**
     * @brief Gets the numerical range of the duty cycle
     *
     * The value passed to @ref pwm must be in this range.
     */
    int pwmRange() const;

    /**
     * @brief Sets the PWM value range for this pin.
     * @return Nearest supported (real) range that was set.
     */
    int setPwmRange(int range);

    /**
     * @brief Gets the range actually used to perform PWM internally.
     *
     * This is the actual PWM resolution.
     */
    int pwmRealRange() const;

    /**
     * @brief Gets the PWM frequency of this pin in Hz.
     */
    int pwmFrequency() const;

    /**
     * @brief Sets the PWM frequency of this pin in Hz.
     * @return The nearest supported frequency that was actually set.
     */
    int setPwmFrequency(int freq);

    /**
     * @brief Registers a callback to be called when the Pin level changes.
     *
     * If `f` is NULL, unregisters the callback.
     */
    void onChange(gpioAlertFuncEx_t f, void *userdata);
};

}

#endif  // PIGPIO_HPP
