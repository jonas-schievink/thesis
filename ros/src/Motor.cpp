#include "Motor.hpp"

#include <iostream>
#include <algorithm>
#include <cstdlib>

using std::cerr;
using std::endl;

Motor::Motor(pigpio::Pin speed_pin, pigpio::Pin dir_pin) :
    m_speed_pin(speed_pin), m_dir_pin(dir_pin) {}

void Motor::set(float speed)
{
    if (speed > 1.0f || speed < -1.0f)
    {
        cerr << "Motor::set invalid speed value " << speed
            << ", clamping to range [-1.0, 1.0]" << endl;
        // NB: std::clamp is C++17
        speed = std::min(1.0f, std::max(-1.0f, speed));
    }

    bool backwards = speed < 0.0f;
    m_dir_pin.digitalWrite(backwards);

    m_speed_pin.pwm((unsigned int) (abs(speed) * 255.0f));
}
