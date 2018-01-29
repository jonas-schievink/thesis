#include "Motor.hpp"

#include <ros/console.h>

#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <stdexcept>
#include <string>

using std::cerr;
using std::endl;
using std::string;
using std::signbit;
using std::chrono::milliseconds;
using std::chrono::duration_cast;

const static float DEFAULT_MAX_ACCEL = 0.3f;
const static float DEFAULT_MAX_DIR_CHANGES = 1.0f;

Motor::Motor(pigpio::Pin speed_pin, pigpio::Pin dir_pin) :
    m_speed_pin(speed_pin), m_dir_pin(dir_pin),
    m_setpoint(0.0f),
    m_actual(0.0f),
    m_maxAccel(DEFAULT_MAX_ACCEL),
    m_dirChangeDelay(1.0f / DEFAULT_MAX_DIR_CHANGES)
{
    m_pwmRange = m_speed_pin.pwmRange();
    ROS_INFO("pwm freq = %d Hz, res = %d, real resolution = %d",
        m_speed_pin.pwmFrequency(),
        m_pwmRange,
        m_speed_pin.pwmRealRange());
}

void Motor::setMaxAcceleration(float maxAccel)
{
    if (maxAccel < 0.0f)
    {
        string msg("maxAccel must be >0, was ");
        msg += maxAccel;
        throw std::invalid_argument(msg);
    }

    m_maxAccel = maxAccel;
}

void Motor::setMaxDirChanges(unsigned int changesPerSecond)
{
    if (changesPerSecond == 0)
    {
        throw std::invalid_argument("direction changes per second must be >0 (is 0)");
    }

    m_dirChangeDelay = 1.0f / (float) changesPerSecond;
}

void Motor::set_direct(float speed)
{
    bool backwards = speed < 0.0f;
    m_dir_pin.digitalWrite(backwards);
    m_speed_pin.pwm((unsigned int) (abs(speed) * float(m_pwmRange)));
}

void Motor::set(float speed)
{
    if (speed > 1.0f || speed < -1.0f)
    {
        string msg("Motor::set speed value must be in range [-1.0, 1.0], is ");
        msg += speed;
        throw std::invalid_argument(msg);
    }

    m_setpoint = speed;
}

void Motor::update()
{
    float diff = m_setpoint - m_actual; // total difference to go
    Motor::UpdateTime now;
    milliseconds delta = duration_cast<milliseconds>(m_lastUpdate - now);
    m_lastUpdate = now;

    ROS_DEBUG("Motor::update: delta = %lld ms, diff = %f", delta.count(), diff);
    //set_direct(m_setpoint);
}

bool Motor::reachedSetPoint() const
{
    // Compare sign bit explicitly in the case of 0.0 vs. -0.0
    // (those would compare equal, but we still need to change direction here)
    return m_actual == m_setpoint && signbit(m_actual) == signbit(m_setpoint);
}
