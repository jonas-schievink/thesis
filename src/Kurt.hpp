#ifndef KURT_HPP
#define KURT_HPP

#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <memory>

/**
 * @brief Exception thrown when a required node parameter is missing.
 */
class MissingNodeParam : public std::exception {
    std::string m_what;

public:
    MissingNodeParam(const char* name);

    const char* what() const noexcept;
};

/**
 * @brief Main robot class
 *
 * Creates and owns robot components (Motors, Controllers, Encoders) and
 * integrates them into ROS.
 */
class Kurt : public hardware_interface::RobotHW
{
public:
    /**
     * @brief Creates a new Kurt instance and loads parameters from ROS.
     */
    Kurt(ros::NodeHandle& nh, ros::NodeHandle& paramHandle);

    // ROS keeps pointers to members, so moving and copying is a big no-no
    Kurt(const Kurt& other) = delete;
    Kurt& operator=(const Kurt& other) = delete;

    /**
     * @brief Gets the main loop frequency at which @ref update must be called.
     * @see getPeriod
     */
    int getFreq() const;

    /**
     * @brief Gets the time between successive calls to @ref update.
     * @see getFreq
     */
    ros::Duration getPeriod() const;

    /**
     * @brief Updates encoders and motor controllers.
     *
     * This must be called at a fixed frequency given by @ref getFreq.
     */
    void update();

    /**
     * @brief Reads the current command and writes it to the controllers.
     *
     * This will not immediately change motor speed. Call @ref update to run the
     * controllers.
     */
    void read();

    /**
     * @brief Writes the current encoder counts to the joint state variables.
     */
    void write();

private:
    /**
     * @brief Calculates the speed of the left wheel in rad/s since the last
     * encoder update.
     */
    double leftSpeed() const;

    /**
     * @brief Calculates the speed of the right wheel in rad/s since the last
     * encoder update.
     */
    double rightSpeed() const;

    bool m_dryrun;
    int m_updateRate;

    std::unique_ptr<Encoder> m_encLeft;
    std::unique_ptr<Encoder> m_encRight;
    std::unique_ptr<Motor> m_motLeft;
    std::unique_ptr<Motor> m_motRight;

    PIDController m_leftController;
    PIDController m_rightController;

    /// @brief Transmits joint/encoder state to ros_control.
    ///
    /// This later ends up on the `/joint_states` topic via the `joint_state_controller`.
    hardware_interface::JointStateInterface m_jointState;
    /// @brief Gets joint velocity commands from ros_control.
    ///
    /// The commands are set by the `diff_drive_controller`.
    hardware_interface::VelocityJointInterface m_jointCtrl;

    /// @brief Velocity commands to the motors.
    double m_cmd[2];
    /// @brief Joint positions from encoders.
    ///
    /// m_pos[0] to m_pos[2] are the left wheels, m_pos[3] to m_pos[5] are the right wheels.
    double m_pos[6];
    /// @brief Joint velocities from encoders.
    ///
    /// m_vel[0] to m_vel[2] are the left wheels, m_vel[3] to m_vel[5] are the right wheels.
    double m_vel[6];
    /// @brief Joint efforts, always 0.
    double m_eff[6];
};

#endif // KURT_HPP
