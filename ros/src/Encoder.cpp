#include "Encoder.hpp"

Encoder::Encoder(int ticksPerTurn, float wheelPerimeter) :
    m_groundDistPerTick(static_cast<double>(wheelPerimeter) / static_cast<double>(ticksPerTurn)) {}

double Encoder::readGroundDist()
{
    return static_cast<double>(read()) * m_groundDistPerTick;
}
