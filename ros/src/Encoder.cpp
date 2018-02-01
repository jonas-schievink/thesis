#include "Encoder.hpp"

Encoder::Encoder(int ticksPerTurn, float wheelPerimeter) :
    m_groundDistPerTick(static_cast<double>(wheelPerimeter) / static_cast<double>(ticksPerTurn)),
    m_groundDist(0), m_speed(0), m_lastUpdate(ros::Time::now()) {}

double Encoder::groundDist()
{
    return m_groundDist;
}

double Encoder::speed()
{
    return m_speed;
}

void Encoder::update()
{
    ros::Time now = ros::Time::now();
    ros::Duration delta = now - m_lastUpdate;
    m_lastUpdate = now;

    int ticks = read();
    m_groundDist = ticks * m_groundDistPerTick;
    m_speed = m_groundDist / delta.toSec();
}
