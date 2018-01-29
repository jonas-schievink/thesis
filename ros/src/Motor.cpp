#include "Motor.hpp"

#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <stdexcept>
#include <string>

using std::cerr;
using std::endl;
using std::string;

Motor::Motor(pigpio::Pin speed_pin, pigpio::Pin dir_pin) :
    m_speed_pin(speed_pin), m_dir_pin(dir_pin), m_setpoint(0.0)
{

}

void Motor::set_direct(float speed)
{
    bool backwards = speed < 0.0f;
    m_dir_pin.digitalWrite(backwards);
    m_speed_pin.pwm((unsigned int) (abs(speed) * 255.0f));
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
    set_direct(m_setpoint);
}
