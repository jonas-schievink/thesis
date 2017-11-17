/**
 * @mainpage
 */

/**
 * @file PiBot.cpp
 * @brief Main project file
 */

#include "PiGPIO.hpp"
#include "Encoder.hpp"

#include <iostream>

#include <unistd.h>

using pigpio::Pin;
using pigpio::PullUpDown;
using std::cout;
using std::endl;

// Pin connected to right motor speed control (PWM)
#define R_SPEED 18

// Right encoder channel A
#define R_ENC_CH_A 8
#define R_ENC_CH_B 7

#define ENC_MAX 100

int main(int argc, char** argv)
{
    // Configure encoder. It needs pull-ups on both channels.
    Pin a(R_ENC_CH_A);
    a.pullupdown(PullUpDown::PullUp);
    Pin b(R_ENC_CH_B);
    b.pullupdown(PullUpDown::PullUp);

    Encoder enc(a, b);
    int total = 0;
    while (true)
    {
        total += enc.read();
        total = total < 0 ? 0 : total;
        total = total > ENC_MAX ? ENC_MAX : total;

        cout << '[';
        for (int i = 0; i <= ENC_MAX; i++)
        {
            if (i == total)
            {
                cout << '#';
            }
            else
            {
                cout << ' ';
            }
        }
        cout << ']' << '\r';
        std::flush(cout);
        usleep(100 * 1000);
    }

    return 0;
}
