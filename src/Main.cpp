#include "Kurt.hpp"
#include "EvdevEncoder.hpp"

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <signal.h>

#include <pigpio.h>

using std::cout;
using std::cerr;
using std::endl;
using std::unique_ptr;

static void process_args(int argc, char** argv)
{
    if (argc > 1)
    {
        cerr << "Invalid command line argument: " << argv[1] << endl;
        cerr << "This node is configured exclusively via ROS node parameters." << endl;
        cerr << "Refer to the default launch file for configuration options." << endl;
        exit(1);
    }
}

/**
 * @brief Checks a pigpio return value for success and exits on errors.
 */
static void checkPigpio(int result)
{
    if (result < 0)
    {
        ROS_ERROR("pigpio error: code %d", result);
        exit(1);
    }
}

static void initPigpio(int sampleRate)
{
    // Save ROS's handlers for SIGINT and SIGTERM
    // The pigpio functionality to stop registering signal handlers isn't yet
    // available in the version shipped by Ubuntun, so we do it manually.
    sighandler_t savedHandlers[2];
    savedHandlers[0] = signal(SIGINT, SIG_DFL);
    savedHandlers[1] = signal(SIGTERM, SIG_DFL);

    // Configure default clock source with sample rate
    checkPigpio(gpioCfgClock(sampleRate, 1 /* PCM = default */, 0));

    checkPigpio(gpioInitialise());

    // Restore ROS signal handlers
    signal(SIGINT, savedHandlers[0]);
    signal(SIGTERM, savedHandlers[1]);
}

int main(int argc, char** argv)
{
    // Note that both pigpio and ROS want to install signal handlers. We *must*
    // initialize ROS before pigpio since we use ROS params to set pigpio
    // configuration.

    ros::init(argc, argv, "kurtberry_pi_node");

    process_args(argc, argv);

    ros::NodeHandle nh;
    ros::NodeHandle paramHandle("~");

    bool debug = paramHandle.param("debug", false);

    if (debug)
    {
        // display ROS_DEBUG output (this spams a lot of controller updates!)
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        {
            ros::console::notifyLoggerLevelsChanged();
        }
    }

    int sampleRate;
    if (!paramHandle.getParam("pwm_sample_rate", sampleRate))
    {
        ROS_ERROR("`pwm_sample_rate` not set");
        return 1;
    }

    initPigpio(sampleRate);

    unique_ptr<Kurt> kurt;
    try
    {
        kurt = unique_ptr<Kurt>(new Kurt(nh, paramHandle));
    }
    catch (const EvdevException& ex)
    {
        ROS_ERROR("couldn't open encoder device: %s", ex.what());
        return 1;
    }
    catch (const MissingNodeParam& ex)
    {
        ROS_ERROR("%s", ex.what());
        return 1;
    }

    controller_manager::ControllerManager cm(&*kurt, nh);

    ros::Rate rate(kurt->getFreq());
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("kurtberry pi setup complete");

    int missedUpdates = 0;
    while (!ros::isShuttingDown())
    {
        kurt->update();
        kurt->read();
        cm.update(ros::Time::now(), kurt->getPeriod());
        kurt->write();

        if (rate.sleep())
        {
            missedUpdates = 0;  // made it in time, reset miss counter
        }
        else
        {
            missedUpdates++;    // this update took too long
        }

        if (missedUpdates >= 3) {
            // The code assumes that `update` is called in regular fixed
            // intervals. If we can't keep this promise, it's an error since all
            // timing gets slightly messed up.
            // Decrease `getFreq` to properly fix this.
            // We accept that this might happen during normal operation due to
            // the Linux scheduler, but will print an error if it happens a few
            // times in a row since that indicates that the system is
            // overloaded.

            const char* note = debug ? " (this is expected in debug mode - set debug param to false and this should go away)" : "";
            float actualHz = 1.0 / rate.cycleTime().toSec();
            ROS_ERROR_THROTTLE(1, "can't keep up with requested update rate of %d Hz! actual rate: %f Hz%s", kurt->getFreq(), actualHz, note);
        }
    }

    while (ros::ok()) {}    // wait until shutdown is complete

    return 0;
}
