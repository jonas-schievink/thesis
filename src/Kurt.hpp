#ifndef KURT_HPP
#define KURT_HPP

#include "Encoder.hpp"
#include "Odometry.hpp"
#include "Motor.hpp"
#include "PIController.hpp"

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <memory>


class Kurt : public hardware_interface::RobotHW
{
public:
    Kurt(ros::NodeHandle& nh);

    Kurt(const Kurt& other) = delete;
    Kurt& operator=(const Kurt& other) = delete;

    ros::Duration getPeriod() const;

    /**
     * @brief Updates encoder counters.
     */
    void update();

    /**
     * @brief Reads the current command and writes it to the motors.
     */
    void read();

    /**
     * @brief Writes the current encoder counts to the joint state.
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

    std::unique_ptr<Encoder> m_encLeft;
    std::unique_ptr<Encoder> m_encRight;
    std::unique_ptr<Odometry> m_odom;
    std::unique_ptr<Motor> m_motLeft;
    std::unique_ptr<Motor> m_motRight;

    PIController m_leftController;
    PIController m_rightController;

    hardware_interface::JointStateInterface m_jointState;
    hardware_interface::VelocityJointInterface m_jointCtrl;

    /// @brief Command to the motors.
    double cmd[2];
    /// @brief Joint positions from encoders.
    double pos[2];
    /// @brief Joint velocities from encoders.
    double vel[2];
    /// @brief Joint efforts, always 0.
    double eff[2];
};

#endif // KURT_HPP
