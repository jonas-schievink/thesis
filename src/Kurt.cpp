#include "Kurt.hpp"

#include "EvdevEncoder.hpp"

using namespace hardware_interface;
using std::unique_ptr;

MissingNodeParam::MissingNodeParam(const char* name)
    : exception()
{
    m_what.append("required node parameter was not defined: ");
    m_what.append(name);
}

const char* MissingNodeParam::what() const noexcept
{
    return m_what.c_str();
}

/**
 * @brief Load a node parameter and throw a @ref MissingNodeParam when unset
 */
template<typename T>
static void param(ros::NodeHandle& p, const char* name, T& variable)
{
    if (!p.getParam(name, variable))
    {
        throw MissingNodeParam(name);
    }
}

Kurt::Kurt(ros::NodeHandle& nh, ros::NodeHandle& p)
{
    cmd[0] = cmd[1] = 0.0;
    pos[0] = pos[1] = 0.0;
    vel[0] = vel[1] = 0.0;
    eff[0] = eff[1] = 0.0;

    param(p, "dryrun", m_dryrun);

    // Setup encoders/odometry
    std::string left_encoder_pattern;
    std::string right_encoder_pattern;
    bool left_encoder_invert;
    bool right_encoder_invert;
    int encoder_wraparound;
    param(p, "left_encoder_pattern", left_encoder_pattern);
    param(p, "right_encoder_pattern", right_encoder_pattern);
    param(p, "left_encoder_invert", left_encoder_invert);
    param(p, "right_encoder_invert", right_encoder_invert);
    param(p, "encoder_wraparound", encoder_wraparound);

    int ticks_per_turn_of_wheel;
    param(p, "ticks_per_turn_of_wheel", ticks_per_turn_of_wheel);

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
    bool left_invert;
    int right_ctrl;
    int right_dir;
    bool right_invert;
    param(p, "left_ctrl_gpio", left_ctrl);
    param(p, "left_dir_gpio", left_dir);
    param(p, "right_ctrl_gpio", right_ctrl);
    param(p, "right_dir_gpio", right_dir);
    param(p, "left_invert", left_invert);
    param(p, "right_invert", right_invert);

    MotorConfig leftCfg(left_ctrl, left_dir);
    param(p, "pwm_freq", leftCfg.pwm_freq);
    param(p, "pwm_range", leftCfg.pwm_range);
    param(p, "max_accel", leftCfg.max_accel);
    param(p, "max_delta_ms", leftCfg.max_delta_ms);
    param(p, "max_dir_changes", leftCfg.max_dir_changes);
    param(p, "deadzone", leftCfg.deadzone);
    leftCfg.invert = left_invert;
    m_motLeft = unique_ptr<Motor>(new Motor(leftCfg));

    MotorConfig rightCfg(leftCfg);
    rightCfg.ctrl_pin = right_ctrl;
    rightCfg.direction_pin = right_dir;
    rightCfg.invert = right_invert;
    m_motRight = unique_ptr<Motor>(new Motor(rightCfg));

    // Configure speed controllers
    float kp;
    float ki;
    float kd;
    float windupLimit;
    param(p, "Kp", kp);
    param(p, "Ki", ki);
    param(p, "Kd", kd);
    param(p, "windup_limit", windupLimit);

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
    JointHandle ctrl_right(m_jointState.getHandle("right_middle_wheel_joint"), &cmd[1]);
    m_jointCtrl.registerHandle(ctrl_right);

    registerInterface(&m_jointCtrl);

    if (m_dryrun)
    {
        ROS_INFO("dryrun, motors will not be controlled");
    }
}

void Kurt::update()
{
    m_encLeft->update();
    m_encRight->update();
    //m_odom->update();

    float leftSpd = m_leftController.update(leftSpeed(), getPeriod().toSec());
    float rightSpd = m_rightController.update(rightSpeed(), getPeriod().toSec());
    m_motLeft->set(leftSpd);
    m_motRight->set(rightSpd);

    m_motLeft->update(m_dryrun);
    m_motRight->update(m_dryrun);

    ROS_DEBUG("L vel = %f rad/s, L cmd = %f rad/s, L spd = %f", leftSpeed(), cmd[0], leftSpd);
    ROS_DEBUG("R vel = %f rad/s, R cmd = %f rad/s, R spd = %f", rightSpeed(), cmd[1], rightSpd);
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

int Kurt::getFreq() const
{
    return 100;
}

ros::Duration Kurt::getPeriod() const
{
    return ros::Duration(1.0f / (float) getFreq());
}
