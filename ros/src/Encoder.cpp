#include "Encoder.hpp"

Encoder::Encoder(int ticksPerTurn, float wheelPerimeter) :
    m_ticksPerTurn(ticksPerTurn),
    m_groundDistPerTick(static_cast<double>(wheelPerimeter) / static_cast<double>(ticksPerTurn)),
    m_revolutions(0.0), m_groundDist(0.0), m_speed(0.0), m_lastUpdate(ros::Time::now()) {}

double Encoder::groundDist() const
{
    return m_groundDist;
}

double Encoder::speed() const
{
    return m_speed;
}

double Encoder::revolutions() const
{
    return m_revolutions;
}

void Encoder::update()
{
    ros::Time now = ros::Time::now();
    ros::Duration delta = now - m_lastUpdate;
    m_lastUpdate = now;

    int ticks = read();
    m_revolutions = ticks / static_cast<double>(m_ticksPerTurn);
    m_groundDist = ticks * m_groundDistPerTick;
    m_speed = m_groundDist / delta.toSec();
}
