#include "Encoder.hpp"

Encoder::Encoder(int ticksPerTurn) :
    m_ticksPerTurn(ticksPerTurn), m_radians(0.0), m_totalRadians(0.0),
    m_lastUpdate(ros::Time::now()) {}

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

double Encoder::radians() const
{
    return m_radians;
}

double Encoder::totalRadians() const
{
    return m_totalRadians;
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

    m_radians = ticks / static_cast<double>(m_ticksPerTurn) * 2.0 * M_PI;
    m_totalRadians += m_radians;

    //if (m_radians < -M_PI) m_radians += 2.0 * M_PI;
    //if (m_radians >  M_PI) m_radians -= 2.0 * M_PI;
}
