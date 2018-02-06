#include "EvdevEncoder.hpp"
#include "Motor.hpp"
#include "Odometry.hpp"
#include "Kurt.hpp"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <controller_manager/controller_manager.h>

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <cstdio>
#include <signal.h>


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

/**
 * @brief Restores the default SIGINT handler.
 *
 * Despite being told explicitly not to do so via
 * `ros::init_options::NoSigintHandler`, ROS will register a signal handler for
 * SIGINT upon node handle creation. That handler, however, doesn't work right
 * and requires pressing Ctrl+C twice.
 *
 * This function will override the ROS handler and restores the default
 * behaviour.
 */
static void stop_hijacking_the_fucking_signal_handler()
{
    signal(SIGINT, SIG_DFL);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kurtberry_pi_node", ros::init_options::NoSigintHandler);

    process_args(argc, argv);

    ros::NodeHandle nh;
    unique_ptr<Kurt> kurt;
    try
    {
        kurt = unique_ptr<Kurt>(new Kurt(nh));
    }
    catch (const EvdevException& ex)
    {
        ROS_ERROR("couldn't open encoder device: %s", ex.what());
        return 1;
    }
    controller_manager::ControllerManager cm(&*kurt, nh);

    stop_hijacking_the_fucking_signal_handler();

    ros::Rate rate(1.0 / kurt->getPeriod().toSec());
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("kurtberry pi setup complete");
    while (ros::ok())
    {
        kurt->update();
        kurt->read();
        cm.update(ros::Time::now(), kurt->getPeriod());
        kurt->write();
        rate.sleep();
    }

    return 0;
}
