#include "GPIOEncoder.hpp"

#include <iostream>

// C callback passed to pigpio, calls `pulse` method
void GPIOEncoder::pulse_helper(int gpio, int level, uint32_t tick, void* user)
{
    GPIOEncoder* enc = static_cast<GPIOEncoder*>(user);
    enc->pulse(gpio, level);
}

void GPIOEncoder::pulse(int pin_num, int level)
{
    // update recorded state
    if (pin_num == m_chan_a.number())
    {
        m_lvlA = level;
    } else {
        m_lvlB = level;
    }

    if (pin_num == m_lastGpio)
    {
        // The channel probably bounced, ignore the event
        // TODO count these like m_relcount to assist in HW debugging
    } else {
        m_lastGpio = pin_num;

        // Now the other channel changed state.
        if (pin_num == m_chan_a.number() && level)
        {
            if (m_lvlB)
            {
                // A rises while B is high -> forward
                m_relcount++;
            }
        }
        else if (pin_num == m_chan_b.number() && level)
        {
            if (m_lvlA)
            {
                // B rises while A is high -> backwards
                m_relcount--;
            }
        }
    }
}

GPIOEncoder::GPIOEncoder(int ticksPerTurn, pigpio::Pin chan_a, pigpio::Pin chan_b)
: Encoder(ticksPerTurn), m_chan_a(chan_a), m_chan_b(chan_b), m_lvlA(0), m_lvlB(0), m_lastGpio(chan_a.number()),
  m_relcount(0) {
    m_chan_a.onChange(GPIOEncoder::pulse_helper, this);
    m_chan_b.onChange(GPIOEncoder::pulse_helper, this);
}

int GPIOEncoder::read()
{
    int count = m_relcount;
    m_relcount = 0;
    return count;
}
