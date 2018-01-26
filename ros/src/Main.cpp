#include "EvdevEncoder.hpp"
#include "Motor.hpp"

#include <ros/console.h>
#include <ros/init.h>

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <cstdio>

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

    try {
        unique_ptr<Encoder> left(new EvdevEncoder("rot_left", 10000, true));
        unique_ptr<Encoder> right(new EvdevEncoder("rot_right", 10000));

        while (true) {
            printf("%40s\r", "");
            printf("--- L: %4d  R: %4d\r", left->read(), right->read());
            fflush(stdout);

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } catch (EvdevException ex) {
        ROS_ERROR("couldn't open encoder device: %s", ex.what());
        return 1;
    }

    return 0;
}
