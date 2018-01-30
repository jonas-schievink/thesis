#include "EvdevEncoder.hpp"
#include "Motor.hpp"
#include "Odometry.hpp"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <cstdio>

// Pin defaults, can be overridden via config
#define PIN_L_CTRL 5
#define PIN_L_DIR  6
#define PIN_R_CTRL 22
#define PIN_R_DIR  23

using std::cout;
using std::endl;
using std::unique_ptr;

/**
 * @brief ROS-Node, sets up subscribers, parameters, etc. and runs the main loop.
 */
class Node
{
    ros::NodeHandle m_handle;
    ros::NodeHandle m_paramHandle;
    ros::Subscriber m_cmd_vel_sub;
    ros::Timer m_timer;
    unique_ptr<RawEncoder> m_encLeft;
    unique_ptr<RawEncoder> m_encRight;
    unique_ptr<Odometry> m_odom;
    unique_ptr<Motor> m_motLeft;
    unique_ptr<Motor> m_motRight;

public:
    Node() : m_handle(), m_paramHandle("~"),
        m_cmd_vel_sub(m_handle.subscribe("cmd_vel", 5, &Node::velCallback, this)),
        m_timer(m_handle.createTimer(ros::Duration(0.02), &Node::timer, this))
    {
        // Setup encoders/odometry
        std::string left_encoder_pattern;
        std::string right_encoder_pattern;
        bool left_encoder_invert;
        bool right_encoder_invert;
        int encoder_wraparound;
        m_paramHandle.param("left_encoder_pattern", left_encoder_pattern, std::string("rot_left"));
        m_paramHandle.param("right_encoder_pattern", right_encoder_pattern, std::string("rot_right"));
        m_paramHandle.param("left_encoder_invert", left_encoder_invert, true);
        m_paramHandle.param("right_encoder_invert", right_encoder_invert, false);
        m_paramHandle.param("encoder_wraparound", encoder_wraparound, 10000);

        m_encLeft = unique_ptr<RawEncoder>(new EvdevEncoder(left_encoder_pattern, encoder_wraparound, left_encoder_invert));
        m_encRight = unique_ptr<RawEncoder>(new EvdevEncoder(right_encoder_pattern, encoder_wraparound, right_encoder_invert));

        int ticks_per_turn_of_wheel;
        m_paramHandle.param("ticks_per_turn_of_wheel", ticks_per_turn_of_wheel, 5000);
        float wheel_perimeter;
        m_paramHandle.param("wheel_perimeter", wheel_perimeter, 0.379f);
        float axis_length;
        m_paramHandle.param("axis_length", axis_length, 0.28f);

        m_odom = unique_ptr<Odometry>(new Odometry(*m_encLeft, *m_encRight, ticks_per_turn_of_wheel, wheel_perimeter, axis_length));

        // Set up motors
        int left_ctrl;
        int left_dir;
        int right_ctrl;
        int right_dir;
        m_paramHandle.param("left_ctrl_gpio", left_ctrl, PIN_L_CTRL);
        m_paramHandle.param("left_dir_gpio", left_dir, PIN_L_DIR);
        m_paramHandle.param("right_ctrl_gpio", right_ctrl, PIN_R_CTRL);
        m_paramHandle.param("right_dir_gpio", right_dir, PIN_R_DIR);

        MotorConfig leftCfg(left_ctrl, left_dir);
        m_motLeft = unique_ptr<Motor>(new Motor(leftCfg));
        MotorConfig rightCfg(right_ctrl, right_dir);
        m_motRight = unique_ptr<Motor>(new Motor(rightCfg));
    }

    /**
     * @brief Called when a message on the `cmd_vel` topic is received.
     */
    void velCallback(const geometry_msgs::Twist& msg)
    {
        ROS_INFO("in cmd_vel callback");
    }

    /**
     * @brief called in regular update interval (>=50 Hz)
     */
    void timer(const ros::TimerEvent& event)
    {
        m_motLeft->update();
        m_motRight->update();
        m_odom->update();
        printf("%40s\r", "");
        printf("    L: %.2f m/s  R: %.2f m/s\r", m_odom->speedLeft(), m_odom->speedRight());
        fflush(stdout);
    }

    void run()
    {
        ros::spin();
    }
};

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
        Node n;
        n.run();
    } catch (const EvdevException& ex) {
        ROS_ERROR("couldn't open encoder device: %s", ex.what());
        return 1;
    }

    return 0;
}
