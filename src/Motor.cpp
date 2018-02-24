#include "Motor.hpp"

#include <ros/console.h>

#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <cinttypes>
#include <stdexcept>
#include <string>

using std::cerr;
using std::endl;
using std::string;
using std::signbit;
using std::chrono::milliseconds;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;

const static float DEFAULT_MAX_ACCEL = 0.3f;
const static float DEFAULT_MAX_DIR_CHANGES = 1.0f;
const static int DEFAULT_PWM_FREQ = 1000; // Hz
const static int DEFAULT_PWM_RANGE = 1000;
const static int DEFAULT_MAX_DELTA = 100; // ms

MotorConfig::MotorConfig(int ctrl_pin, int dir_pin) :
    ctrl_pin(ctrl_pin),
    direction_pin(dir_pin),
    pwm_freq(DEFAULT_PWM_FREQ),
    pwm_range(DEFAULT_PWM_RANGE),
    max_delta_ms(DEFAULT_MAX_DELTA)
{
}

Motor::Motor(MotorConfig config) :
    m_speed_pin(config.ctrl_pin),
    m_dir_pin(config.direction_pin),
    m_config(config),
    m_setpoint(0.0f),
    m_actual(0.0f),
    m_dirChangeDelay(1.0f / config.max_dir_changes)
{
    m_lastUpdate = m_lastDirChange = high_resolution_clock::now();
    m_speed_pin.setPwmFrequency(m_config.pwm_freq);
    m_pwmRange = m_speed_pin.setPwmRange(m_config.pwm_range);
    ROS_INFO("requested pwm freq = %d Hz, range = %d", m_config.pwm_freq, m_config.pwm_range);
    ROS_INFO("actual    pwm freq = %d Hz, range = %d", m_speed_pin.pwmFrequency(), m_pwmRange);
}

void Motor::setDirect(float speed)
{
    if (fabs(speed) > 0.001f) {
        bool backwards = speed < 0.0f;
        m_dir_pin.digitalWrite(backwards);
    }
    m_speed_pin.pwm((unsigned int) (fabs(speed) * float(m_pwmRange)));
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
    float diff = m_setpoint - m_actual;
    Motor::UpdateTime now = high_resolution_clock::now();
    int64_t delta_ms = duration_cast<milliseconds>(now - m_lastUpdate).count();
    m_lastUpdate = now;

    if (delta_ms > m_config.max_delta_ms)
    {
        ROS_WARN_STREAM_THROTTLE(1, "delay between Motor::update call too high! time delta = " << delta_ms << " ms");
        delta_ms = m_config.max_delta_ms;
    }

    float delta = static_cast<float>(delta_ms) / 1000.0f;
    // acceleration we'd like to apply during the last update period
    float accel = diff / delta;

    // FIXME: replace with actual ramp-up logic
    setDirect(m_setpoint);
}

bool Motor::reachedSetPoint() const
{
    // Compare sign bit explicitly in the case of 0.0 vs. -0.0
    // (those would compare equal, but we still need to change direction here)
    return m_actual == m_setpoint && signbit(m_actual) == signbit(m_setpoint);
}
