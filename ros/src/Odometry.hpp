#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include <ros/node_handle.h>

class Encoder;

/**
 * @brief Relative robot movement (linear and angular)
 */
struct OdomMovement {
    /// @brief Forward movement in meters.
    double forward;
    /// @brief Rightward movement in meters (positive values = to the right).
    double rightward;
    /// @brief Clockwise rotation (to the right) in Radians.
    double angular;
};

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

    OdomMovement calcMovement();

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
