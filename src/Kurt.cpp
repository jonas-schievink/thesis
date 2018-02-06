#include "Kurt.hpp"

#include "EvdevEncoder.hpp"

using namespace hardware_interface;
using std::unique_ptr;

// Pin defaults, can be overridden via config
#define PIN_L_CTRL 5
#define PIN_L_DIR  6
#define PIN_R_CTRL 22
#define PIN_R_DIR  23

Kurt::Kurt(ros::NodeHandle& nh)
{
    cmd[0] = cmd[1] = 0.0;
    pos[0] = pos[1] = 0.0;
    vel[0] = vel[1] = 0.0;
    eff[0] = eff[1] = 0.0;

    ros::NodeHandle paramHandle("~");

    // Setup encoders/odometry
    std::string left_encoder_pattern;
    std::string right_encoder_pattern;
    bool left_encoder_invert;
    bool right_encoder_invert;
    int encoder_wraparound;
    paramHandle.param("left_encoder_pattern", left_encoder_pattern, std::string("rot_left"));
    paramHandle.param("right_encoder_pattern", right_encoder_pattern, std::string("rot_right"));
    paramHandle.param("left_encoder_invert", left_encoder_invert, true);
    paramHandle.param("right_encoder_invert", right_encoder_invert, false);
    paramHandle.param("encoder_wraparound", encoder_wraparound, 10000);

    int ticks_per_turn_of_wheel;
    paramHandle.param("ticks_per_turn_of_wheel", ticks_per_turn_of_wheel, 5000);

    m_encLeft = unique_ptr<Encoder>(new EvdevEncoder(
        ticks_per_turn_of_wheel,
        left_encoder_pattern,
        encoder_wraparound,
        left_encoder_invert
    ));
    m_encRight = unique_ptr<Encoder>(new EvdevEncoder(
        ticks_per_turn_of_wheel,
        right_encoder_pattern,
        encoder_wraparound,
        right_encoder_invert
    ));

    // Set up motors
    int left_ctrl;
    int left_dir;
    int right_ctrl;
    int right_dir;
    paramHandle.param("left_ctrl_gpio", left_ctrl, PIN_L_CTRL);
    paramHandle.param("left_dir_gpio", left_dir, PIN_L_DIR);
    paramHandle.param("right_ctrl_gpio", right_ctrl, PIN_R_CTRL);
    paramHandle.param("right_dir_gpio", right_dir, PIN_R_DIR);

    MotorConfig leftCfg(left_ctrl, left_dir);
    m_motLeft = unique_ptr<Motor>(new Motor(leftCfg));
    MotorConfig rightCfg(right_ctrl, right_dir);
    m_motRight = unique_ptr<Motor>(new Motor(rightCfg));

    // Connect to PID topics
    m_leftSetpoint = nh.advertise<std_msgs::Float64>("/left_pid/setpoint", 1);
    m_rightSetpoint = nh.advertise<std_msgs::Float64>("/right_pid/setpoint", 1);
    m_leftState = nh.advertise<std_msgs::Float64>("/left_pid/state", 1);
    m_rightState = nh.advertise<std_msgs::Float64>("/right_pid/state", 1);
    m_leftEffort = nh.subscribe("/left_pid/control_effort", 1, &Kurt::leftCtrl, this);
    m_rightEffort = nh.subscribe("/right_pid/control_effort", 1, &Kurt::rightCtrl, this);

    // Register the joint state interface.
    // It will report the current joint state as read from the encoders to ROS.
    JointStateHandle state_left("left_middle_wheel_joint", &pos[0], &vel[0], &eff[0]);
    m_jointState.registerHandle(state_left);
    JointStateHandle state_right("right_middle_wheel_joint", &pos[1], &vel[1], &eff[1]);
    m_jointState.registerHandle(state_right);

    registerInterface(&m_jointState);

    // Register joint control interface.
    JointHandle ctrl_left(m_jointState.getHandle("left_middle_wheel_joint"), &cmd[0]);
    m_jointCtrl.registerHandle(ctrl_left);
    JointHandle ctrl_right(m_jointState.getHandle("right_middle_wheel_joint"), &cmd[0]);
    m_jointCtrl.registerHandle(ctrl_right);

    registerInterface(&m_jointCtrl);
}

void Kurt::update()
{
    m_encLeft->update();
    m_encRight->update();
    m_motLeft->update();
    m_motRight->update();
}

void Kurt::read() const
{
    // apply cmd[0]/cmd[1] to motor controllers
    std_msgs::Float64 setpoint;
    setpoint.data = cmd[0];
    m_leftSetpoint.publish(setpoint);
    setpoint.data = cmd[1];
    m_rightSetpoint.publish(setpoint);

    // also send state in radians / second so the controller can do work
    std_msgs::Float64 state;
    state.data = m_encLeft->radians() / getPeriod().toSec();
    m_leftState.publish(state);
    state.data = m_encRight->radians() / getPeriod().toSec();
    m_rightState.publish(state);
}

void Kurt::write()
{
    // write wheel rotations to vars
    pos[0] = m_encLeft->totalRadians();
    pos[1] = m_encRight->totalRadians();
    vel[0] = pos[0] / getPeriod().toSec();
    vel[1] = pos[1] / getPeriod().toSec();
}

void Kurt::leftCtrl(const std_msgs::Float64& msg)
{
    m_motLeft->set(msg.data);
}

void Kurt::rightCtrl(const std_msgs::Float64& msg)
{
    m_motRight->set(msg.data);
}

ros::Duration Kurt::getPeriod() const
{
    return ros::Duration(0.01);
}
