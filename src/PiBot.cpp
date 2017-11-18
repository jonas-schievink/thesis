/**
 * @mainpage
 */

/**
 * @file PiBot.cpp
 * @brief Main project file
 */

#include "PiGPIO.hpp"
#include "Encoder.hpp"
#include "Motor.hpp"

#include <iostream>

#include <unistd.h>

using pigpio::Pin;
using pigpio::PullUpDown;
using std::cout;
using std::endl;

// Pin connected to right motor speed control (PWM)
#define R_SPEED 18
#define R_DIR   23  // unused!

// Right encoder channel A
#define R_ENC_CH_A 8
#define R_ENC_CH_B 7

#define ENC_MAX 100

int main(int argc, char** argv)
{
    // Configure motor
    Pin spd(R_SPEED);
    Pin dir(R_DIR);
    spd.output();
    dir.output();
    spd.digitalWrite(true);
    //spd.pwm(100);
    //Motor motor(spd, dir);
    //motor.set(0.5f);

    // Configure encoder. It needs pull-ups on both channels.
    Pin a(R_ENC_CH_A);
    Pin b(R_ENC_CH_B);
    a.pullupdown(PullUpDown::PullUp);
    b.pullupdown(PullUpDown::PullUp);
    Encoder enc(a, b);

    int total = ENC_MAX / 2;
    while (true)
    {
        total += enc.read();
        total = total < 0 ? 0 : total;
        total = total > ENC_MAX ? ENC_MAX : total;

        // total is now in range [0...ENC_MAX]
        float speed = total / ENC_MAX - 0.5f;
        //motor.set(speed);

        cout << '[';
        for (int i = 0; i <= ENC_MAX; i++)
        {
            if (i == total)
            {
                cout << '#';
            }
            else
            {
                cout << '-';
            }
        }
        cout << ']';
        std::flush(cout);
        usleep(100 * 1000);
        cout << '\r';
    }

    return 0;
}
