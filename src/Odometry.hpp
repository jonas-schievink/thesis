#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include <ros/node_handle.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

class Encoder;

/**
 * @brief Calculates odometry messages from left/right encoders.
 */
class Odometry {
    ros::NodeHandle m_node;
    Encoder& m_left;
    Encoder& m_right;
    /// @brief Medium wheel distance across their axis in meters.
    float m_axisLength;
    /// @brief `/odom` publisher.
    ros::Publisher m_odomPub;
    /// @brief `/joint_states` publisher.
    ros::Publisher m_jointPub;

    double m_x;
    double m_y;
    double m_theta;

    /// @brief Rotation angle of left wheel (-Pi .. +Pi)
    double m_leftWheel;
    /// @brief Rotation angle of right wheel (-Pi .. +Pi)
    double m_rightWheel;

    void calcJointState(sensor_msgs::JointState*);

    /**
     * @brief populates pose and twist of the Odometry message.
     */
    void calcMovement(nav_msgs::Odometry*);

public:
    /**
     * @brief Creates a new odometry processor
     * @param node ROS node handle, used to create publishers
     * @param left Left wheel encoder
     * @param right Right wheel encoder
     * @param axisLength Wheel distance in meters
     */
    Odometry(ros::NodeHandle node, Encoder& left, Encoder& right, float axisLength);

    /**
     * @brief Build and send odometry updates using the current encoder values.
     *
     * @ref Encoder::update must be called in the same intervals as this method.
     */
    void update();
};

#endif // ODOMETRY_HPP