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
#include <cstdlib>
#include <signal.h>

#include <pigpio.h>

using std::cout;
using std::endl;
using std::unique_ptr;

static void on_exit()
{
    static bool exited = false;   // `on_exit` called?

    if (exited)
    {
        return;
    }

    exited = true;

    cout << "on_exit(): shutting down pigpio" << endl;

    // pigpio needs to be properly shut down or it might break everything until
    // the Pi is rebooted.
    gpioTerminate();
}

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
    // Both pigpio and ROS want to install signal handlers, so a little
    // "initialization dance" is needed. We'll use ROS signal handlers, which
    // will cause the main loop and program to exit. pigpio will then be shut
    // down using an `atexit` handler and isn't allowed to catch signals.
    gpioInitialise();
    atexit(on_exit);

    // ROS will overwrite pigpio's signal handlers
    ros::init(argc, argv, "kurtberry_pi_node");

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
    catch (const ConfigException& ex)
    {
        ROS_ERROR("exiting due to configuration error: %s", ex.what());
        return 1;
    }

    controller_manager::ControllerManager cm(&*kurt, nh);

    ros::Rate rate(1.0 / kurt->getPeriod().toSec());
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("kurtberry pi setup complete");

    while (!ros::isShuttingDown())
    {
        kurt->update();
        kurt->read();
        cm.update(ros::Time::now(), kurt->getPeriod());
        kurt->write();
        rate.sleep();
    }

    // Destructors of ROS objects wait *very long* until they return for some
    // reason. Call our exit handler before that happens. Its effects only
    // happen once.
    on_exit();

    while (ros::ok()) {}    // wait until shutdown is complete

    return 0;
}
