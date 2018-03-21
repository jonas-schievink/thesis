#include "PIDController.hpp"

#include <ros/console.h>

#include <algorithm>

PIDController::PIDController(float kp, float ki, float kd, float integralLimit) :
m_setpoint(0.0f), m_effort(0.0f), m_integral(0.0f), m_lastError(0.0f),
m_kp(kp), m_ki(ki), m_kd(kd), m_integralLimit(integralLimit) {
    ROS_INFO("PIDController ctor: kp=%-5.3f ki=%-5.3f kd=%-5.3f ilimit=%-5.3f", kp, ki, kd, integralLimit);
}

PIDController::PIDController() : PIDController(0, 0, 0, 0) {}

void PIDController::setSetpoint(float setpoint)
{
    m_setpoint = setpoint;
}

float PIDController::update(float actual, float delta)
{
    float error = m_setpoint - actual;

    m_integral += error * delta;
    m_integral = std::min(m_integralLimit, m_integral);
    m_integral = std::max(-m_integralLimit, m_integral);

    float propPortion  = error * m_kp;
    float intPortion   = m_integral * m_ki;
    float derivPortion = (error - m_lastError) / delta * m_kd;
    m_lastError = error;

    m_effort += propPortion + intPortion + derivPortion;

    // clamp
    m_effort = std::min(MAX_EFFORT, m_effort);
    m_effort = std::max(MIN_EFFORT, m_effort);

    ROS_DEBUG("PIDController::update(): setpoint=%-5.3f actual=%-5.3f error=%-5.3f delta=%-5.3f integral=%-5.3f propPortion=%-5.3f intPortion=%-5.3f derivPortion=%-5.3f effort=%-5.3f", m_setpoint, actual, error, delta, m_integral, propPortion, intPortion, derivPortion, m_effort);
    return m_effort;
}
