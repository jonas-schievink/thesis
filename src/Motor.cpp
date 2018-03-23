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

const static float DEFAULT_MAX_ACCEL = 0.1f;
const static float DEFAULT_MAX_DIR_CHANGES = 1.0f;
const static int DEFAULT_PWM_FREQ = 2000; // Hz
const static int DEFAULT_PWM_RANGE = 100;
const static int DEFAULT_MAX_DELTA = 100; // ms
const static int DEFAULT_DEADZONE = 0.0f;

MotorConfig::MotorConfig(int ctrl_pin, int dir_pin) :
    ctrl_pin(ctrl_pin),
    direction_pin(dir_pin),
    pwm_freq(DEFAULT_PWM_FREQ),
    pwm_range(DEFAULT_PWM_RANGE),
    max_delta_ms(DEFAULT_MAX_DELTA),
    max_accel(DEFAULT_MAX_ACCEL),
    max_dir_changes(DEFAULT_MAX_DIR_CHANGES),
    deadzone(DEFAULT_DEADZONE),
    invert(false)
{
}

Motor::Motor(MotorConfig config) :
    m_speed_pin(config.ctrl_pin),
    m_dir_pin(config.direction_pin),
    m_config(config),
    m_setpoint(0.0f),
    m_actual(0.0f),
    m_dirChangeDelay(1.0f / config.max_dir_changes),
    m_firstUpdate(true)
{
    ROS_INFO("configuring new Motor instance:");
    ROS_INFO("  ctrl pin: %d", config.ctrl_pin);
    ROS_INFO("  dir pin:  %d", config.direction_pin);
    ROS_INFO("  max dir changes/s:  %f", config.max_dir_changes);
    ROS_INFO("  = dir change delay: %f", m_dirChangeDelay);
    ROS_INFO("  max delta time:     %ims", config.max_delta_ms);
    ROS_INFO("  max acceleration:   %f", config.max_accel);
    ROS_INFO("  deadzone:           %f", config.deadzone);

    m_speed_pin.setPwmFrequency(m_config.pwm_freq);
    m_pwmRange = m_speed_pin.setPwmRange(m_config.pwm_range);
    ROS_INFO("requested pwm freq = %d Hz, range = %d", m_config.pwm_freq, m_config.pwm_range);
    ROS_INFO("actual    pwm freq = %d Hz, range = %d", m_speed_pin.pwmFrequency(), m_pwmRange);
}

void Motor::setDirect(float speed)
{
    if (fabs(speed) > 0.001f)
    {
        // Only update direction when speed is large-ish. Prevent needless
        // direction changes when speed oscillates around 0.
        bool backwards = speed < 0.0f;
        m_dir_pin.digitalWrite(backwards ^ m_config.invert);
    }

    unsigned int duty;
    if (fabs(speed) <= m_config.deadzone)
    {
        // Inside deadzone -> Clamp to zero
        duty = 0;
    }
    else
    {
        duty = (unsigned int) (fabs(speed) * float(m_pwmRange));
    }

    m_speed_pin.pwm(duty);
    m_actual = speed;
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

void Motor::update(bool dryrun)
{
    if (m_firstUpdate)
    {
        // do nothing, just set the times to have a reference
        m_lastUpdate = m_lastDirChange = ros::Time::now();
        m_firstUpdate = false;
        return;
    }

    float diff = m_setpoint - m_actual;
    ros::Time now = ros::Time::now();
    ros::Duration delta = now - m_lastUpdate;
    m_lastUpdate = now;

    int64_t delta_ms = delta.toNSec() / 1000000;
    if (delta_ms > m_config.max_delta_ms)
    {
        ROS_WARN_STREAM_THROTTLE(1, "delay between Motor::update calls too high! time delta = " << delta_ms << " ms");
        delta_ms = m_config.max_delta_ms;
    }

    // acceleration we'd like to apply (might be negative)
    float accel = diff / delta.toSec();
    // clamp to maximum, keeping the sign
    accel = copysign(std::min(std::abs(accel), m_config.max_accel), accel);

    // TODO direction change limit

    float speed = std::min(std::max(m_actual + accel * static_cast<float>(delta.toSec()), -1.0f), 1.0f);
    m_actual = speed;
    ROS_DEBUG("Motor::update(): setpoint=%-5.2f diff=%-5.2f delta=%-5.3fs accel=%-5.2f spd=%-5.2f", m_setpoint, diff, delta.toSec(), accel, speed);

    if (!dryrun)
    {
        setDirect(speed);
    }
}

bool Motor::reachedSetPoint() const
{
    // Compare sign bit explicitly in the case of 0.0 vs. -0.0
    // (those would compare equal, but we still need to change direction here)
    return m_actual == m_setpoint && signbit(m_actual) == signbit(m_setpoint);
}
