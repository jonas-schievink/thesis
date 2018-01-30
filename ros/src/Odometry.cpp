#include "Odometry.hpp"

Odometry::Odometry(RawEncoder& left, RawEncoder& right, int ticksPerTurn, float wheelPerimeter, float axisLength) :
    m_left(left), m_right(right), m_ticksPerTurn(ticksPerTurn), m_wheelPerimeter(wheelPerimeter), m_axisLength(axisLength)
{

}

float Odometry::speedLeft() const
{
    return 0.0f;
}

float Odometry::speedRight() const
{
    return 0.0f;
}

void Odometry::update()
{

}
