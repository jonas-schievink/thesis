/**
 * @file GPIOEncoder.hpp
 * @brief Allows reading rotary encoders attached to the GPIO pins
 */

#ifndef GPIOENCODER_HPP
#define GPIOENCODER_HPP

#include "PiGPIO.hpp"

// TODO Document expected encoder behaviour wrt channel signals #A and #B

/**
 * @brief A rotary encoder attached to two GPIO pins.
 */
class Encoder {
    pigpio::Pin m_chan_a;
    pigpio::Pin m_chan_b;
    // last recorded level of A channel
    int m_lvlA;
    // last recorded level of B channel
    int m_lvlB;
    // last GPIO that changed state. used for debouncing.
    int m_lastGpio;
    // current tick counter
    int m_relcount;

    static void pulse_helper(int gpio, int level, uint32_t tick, void* user);

    void pulse(int pin_num, int level);

public:
    /**
     * @brief Create a new `Encoder` attached to 2 pins.
     *
     * The pins will not be reconfigured. You must configure pullups as
     * appropriate before creating the `Encoder`.
     */
    Encoder(pigpio::Pin chan_a, pigpio::Pin chan_b);

    /**
     * @brief Reads the number of counts since the last call of `read`
     *
     * If this is the first call to `read`, returns the number of counts since
     * the creation of the `Encoder`.
     */
    int read();
};

#endif  // GPIOENCODER_HPP
