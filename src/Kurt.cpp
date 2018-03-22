#include "Kurt.hpp"

#include "EvdevEncoder.hpp"

using namespace hardware_interface;
using std::unique_ptr;

// Pin defaults, can be overridden via config
#define PIN_L_CTRL 22
#define PIN_L_DIR  23
#define PIN_R_CTRL 5
#define PIN_R_DIR  6

ConfigException::ConfigException(const char* msg)
    : runtime_error(msg) {}

Kurt::Kurt(ros::NodeHandle& nh, ros::NodeHandle& paramHandle)
{
    cmd[0] = cmd[1] = 0.0;
    pos[0] = pos[1] = 0.0;
    vel[0] = vel[1] = 0.0;
    eff[0] = eff[1] = 0.0;

    paramHandle.param("dryrun", m_dryrun, false);
    if (m_dryrun)
    {
        ROS_INFO("dryrun, motors will not be controlled");
    }

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
    paramHandle.param("encoder_wraparound", encoder_wraparound, 100000);

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
    m_odom = unique_ptr<Odometry>(new Odometry(nh, *m_encLeft, *m_encRight, 1.0f));

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
    paramHandle.getParam("pwm_freq", leftCfg.pwm_freq);
    paramHandle.getParam("pwm_range", leftCfg.pwm_range);
    paramHandle.getParam("max_delta_ms", leftCfg.max_delta_ms);
    paramHandle.getParam("max_accel", leftCfg.max_accel);
    paramHandle.getParam("max_dir_changes", leftCfg.max_dir_changes);
    paramHandle.getParam("deadzone", leftCfg.deadzone);
    m_motLeft = unique_ptr<Motor>(new Motor(leftCfg));

    MotorConfig rightCfg(leftCfg);
    rightCfg.ctrl_pin = right_ctrl;
    rightCfg.direction_pin = right_dir;
    m_motRight = unique_ptr<Motor>(new Motor(rightCfg));

    // Configure speed controllers
    float kp;
    float ki;
    float kd;
    float windupLimit;
    bool pidParamsPresent = true;
    pidParamsPresent &= paramHandle.getParam("Kp", kp);
    pidParamsPresent &= paramHandle.getParam("Ki", ki);
    pidParamsPresent &= paramHandle.getParam("Kd", kd);
    pidParamsPresent &= paramHandle.getParam("windup_limit", windupLimit);
    if (!pidParamsPresent)
    {
        throw ConfigException("Missing node parameters! The `Kp`, `Ki`, `Kd` and `windup_limit` parameters are required.");
    }

    m_leftController = PIDController(kp, ki, kd, windupLimit);
    m_rightController = PIDController(kp, ki, kd, windupLimit);

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
    //m_odom->update();

    float leftEffort = m_leftController.update(leftSpeed(), getPeriod().toSec());
    float rightEffort = m_rightController.update(rightSpeed(), getPeriod().toSec());
    m_motLeft->set(leftEffort);
    m_motRight->set(rightEffort);

    m_motLeft->update(m_dryrun);
    m_motRight->update(m_dryrun);

    ROS_DEBUG("L vel = %f rad/s, L cmd = %f rad/s, L effort = %f", leftSpeed(), cmd[0], leftEffort);
}

void Kurt::read()
{
    // read motor commands from ROS and apply them
    m_leftController.setSetpoint(cmd[0]);
    m_rightController.setSetpoint(cmd[1]);
}

void Kurt::write()
{
    // write wheel rotations to vars
    pos[0] = m_encLeft->totalRadians();
    pos[1] = m_encRight->totalRadians();
    vel[0] = leftSpeed();
    vel[1] = rightSpeed();
}

double Kurt::leftSpeed() const
{
    return m_encLeft->radians() / getPeriod().toSec();
}

double Kurt::rightSpeed() const
{
    return m_encRight->radians() / getPeriod().toSec();
}

ros::Duration Kurt::getPeriod() const
{
    return ros::Duration(0.05);
}
