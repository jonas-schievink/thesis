#include "PIController.hpp"

#include <ros/console.h>

#include <algorithm>

PIController::PIController(float kp, float ki, float integralLimit) :
    m_setpoint(0.0f), m_effort(0.0f), m_integral(0.0f),
    m_kp(kp), m_ki(ki), m_integralLimit(integralLimit) {}

PIController::PIController() : PIController(0, 0, 0) {}

void PIController::setSetpoint(float setpoint)
{
    m_setpoint = setpoint;
}

float PIController::update(float actual, float delta)
{
    float error = m_setpoint - actual;

    m_integral += error * delta;
    m_integral = std::min(m_integralLimit, m_integral);
    m_integral = std::max(-m_integralLimit, m_integral);

    float effort = (error * m_kp) + (m_integral * m_ki);
    m_effort += effort;

    // clamp
    m_effort = std::min(MAX_EFFORT, m_effort);
    m_effort = std::max(MIN_EFFORT, m_effort);

    ROS_DEBUG("PIController::update: setpoint=%f actual=%f error=%f delta=%f integral=%f effort=%f", m_setpoint, actual, error, delta, m_integral, m_effort);
    return m_effort;
}
