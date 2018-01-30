#include "EvdevEncoder.hpp"
#include "Motor.hpp"

#include <ros/console.h>
#include <ros/init.h>

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <cstdio>

#define PIN_L_CTRL 5
#define PIN_L_DIR  6
#define PIN_R_CTRL 22
#define PIN_R_DIR  23

using std::cout;
using std::endl;
using std::unique_ptr;

static void process_args(int argc, char** argv)
{
    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];

        if (arg == "--debug")
        {
            if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
                ros::console::notifyLoggerLevelsChanged();
            }
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kurtberry_pi_node");

    process_args(argc, argv);

    unique_ptr<RawEncoder> encLeft;
    unique_ptr<RawEncoder> encRight;
    int l_sum = 0;
    int r_sum = 0;

    try {
        encLeft = unique_ptr<RawEncoder>(new EvdevEncoder("rot_left", 10000, true));
        encRight = unique_ptr<RawEncoder>(new EvdevEncoder("rot_right", 10000));
    } catch (EvdevException ex) {
        ROS_ERROR("couldn't open encoder device: %s", ex.what());
        return 1;
    }

    MotorConfig leftCfg(PIN_L_CTRL, PIN_L_DIR);
    Motor left(leftCfg);

    while (true) {
        printf("%40s\r", "");
        l_sum += encLeft->read();
        r_sum += encRight->read();
        printf("    L: %4d  R: %4d\r", l_sum, r_sum);
        fflush(stdout);

        left.update();

        // FIXME: Use a fixed-rate loop instead
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return 0;
}
