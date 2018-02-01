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
#include <signal.h>

// Pin defaults, can be overridden via config
#define PIN_L_CTRL 5
#define PIN_L_DIR  6
#define PIN_R_CTRL 22
#define PIN_R_DIR  23

using std::cout;
using std::endl;
using std::unique_ptr;

/**
 * @brief Frequency of the node update timer.
 */
const static double TIMER_FREQ = 50; // Hz
const static double TIMER_INTERVAL = 1.0 / TIMER_FREQ; // s

/**
 * @brief Safety timeout after which velocity will be reset if no cmd_vel is received.
 */
const static double CMD_VEL_TIMEOUT = 0.6; // s

/**
 * @brief ROS message queue size for the cmd_vel topic
 *
 * Since we're only interested in the most recent message, we can set this to 1
 * to save a bit of space and improve controller latency.
 */
const static int CMD_VEL_QUEUE_SIZE = 1;

/**
 * @brief ROS-Node, sets up subscribers, parameters, etc. and runs the main loop.
 */
class Node
{
    ros::NodeHandle m_handle;
    ros::NodeHandle m_paramHandle;
    ros::Subscriber m_cmd_vel_sub;
    ros::Timer m_timer;
    /// @brief Time of last msg received on `cmd_vel` topic.
    ros::Time m_lastCmdVel;
    /// @brief cmd_vel message timeout after which velocity will be reset to 0.
    ros::Duration m_cmdVelTimeout;
    float m_axisLength;
    unique_ptr<Encoder> m_encLeft;
    unique_ptr<Encoder> m_encRight;
    unique_ptr<Odometry> m_odom;
    unique_ptr<Motor> m_motLeft;
    unique_ptr<Motor> m_motRight;

public:
    Node() : m_handle(), m_paramHandle("~"),
        m_cmd_vel_sub(m_handle.subscribe("cmd_vel", CMD_VEL_QUEUE_SIZE, &Node::velCallback, this)),
        m_timer(m_handle.createTimer(ros::Duration(TIMER_INTERVAL), &Node::timer, this)),
        m_lastCmdVel(ros::Time::now()), m_cmdVelTimeout(CMD_VEL_TIMEOUT)
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

        int ticks_per_turn_of_wheel;
        m_paramHandle.param("ticks_per_turn_of_wheel", ticks_per_turn_of_wheel, 5000);
        float wheel_perimeter;
        m_paramHandle.param("wheel_perimeter", wheel_perimeter, 0.379f);
        m_paramHandle.param("axis_length", m_axisLength, 0.28f);

        m_encLeft = unique_ptr<Encoder>(new EvdevEncoder(
            ticks_per_turn_of_wheel,
            wheel_perimeter,
            left_encoder_pattern,
            encoder_wraparound,
            left_encoder_invert
        ));
        m_encRight = unique_ptr<Encoder>(new EvdevEncoder(
            ticks_per_turn_of_wheel,
            wheel_perimeter,
            right_encoder_pattern,
            encoder_wraparound,
            right_encoder_invert
        ));

        m_odom = unique_ptr<Odometry>(new Odometry(*m_encLeft, *m_encRight, ticks_per_turn_of_wheel, wheel_perimeter, m_axisLength));

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

    Node(const Node& other) = delete;
    Node& operator=(const Node& other) = delete;

    /**
     * @brief Called when a message on the `cmd_vel` topic is received.
     *
     * Note that a publisher might suddenly disappear, in which case we should
     * reset velocity to 0 to prevent accidents.
     *
     * The original node used a timeout of 0.6 seconds after which velocity was
     * reset. All remote control/teleop nodes seem to send update faster than
     * that, so this seems fine.
     *
     * However, since KURT is so fast, we might want to reduce this timeout.
     */
    void velCallback(const geometry_msgs::Twist& msg)
    {
        m_lastCmdVel = ros::Time::now();

        // Convert to left/right wheel speed.
        // `angular.z` is the amount the robot should turn, `linear.x` is the
        // linear speed in forward direction (since this message is always in
        // robot-local coordinates).
        float left  = msg.linear.x - m_axisLength * msg.angular.z;
        float right = msg.linear.x + m_axisLength * msg.angular.z;
        ROS_DEBUG("cmd_vel: %.2f | %.2f", left, right);
    }

    /**
     * @brief Called in regular intervals (>=50 Hz), updates complete node state.
     *
     * This will update odometry calculation and the motor controller, possibly
     * publishing an odometry message.
     */
    void timer(const ros::TimerEvent& event)
    {
        ros::Duration lastCmdDelay = ros::Time::now() - m_lastCmdVel;
        if (lastCmdDelay > m_cmdVelTimeout)
        {
            ROS_DEBUG("no cmd_vel received, resetting speed to 0");
        }

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

    try {
        Node n;
        stop_hijacking_the_fucking_signal_handler();
        n.run();
    } catch (const EvdevException& ex) {
        ROS_ERROR("couldn't open encoder device: %s", ex.what());
        return 1;
    }

    return 0;
}
