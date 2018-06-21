#include "Encoder.hpp"

Encoder::Encoder(int ticksPerTurn) :
    m_ticksPerTurn(ticksPerTurn), m_radians(0.0), m_totalRadians(0.0),
    m_lastUpdate(ros::Time::now()) {}

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

    m_radians = ticks / static_cast<double>(m_ticksPerTurn) * 2.0 * M_PI;
    m_totalRadians += m_radians;
}
