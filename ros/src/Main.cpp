#include "EvdevEncoder.hpp"
#include "Motor.hpp"

#include <ros/console.h>
#include <ros/init.h>

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>

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
        unique_ptr<Encoder> left(new EvdevEncoder("rot_left"));
        unique_ptr<Encoder> right(new EvdevEncoder("rot_right"));

        while (true) {
            cout << left->read() << endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
    } catch (EvdevException ex) {
        ROS_ERROR("couldn't open encoder device: %s", ex.what());
        return 1;
    }

    return 0;
}
