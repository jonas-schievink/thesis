#include "Odometry.hpp"

#include <nav_msgs/Odometry.h>

Odometry::Odometry(ros::NodeHandle node, Encoder& left, Encoder& right, float axisLength) :
    m_node(node), m_left(left), m_right(right), m_axisLength(axisLength)
{
    m_odomPub = m_node.advertise<nav_msgs::Odometry>("/odom", 1);
}

void Odometry::update()
{
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();

    // Set pose reference frame, can be any frame with a fixed connection to the
    // robot.
    odom.header.frame_id = "odom_combined";
    // Set twist reference frame, must be the center of the reported rotation.
    odom.child_frame_id = "base_footprint";

    OdomMovement movement = calcMovement();
    // TODO: Fill in pose values.
}

OdomMovement Odometry::calcMovement()
{
    // TODO: Perform odometry calculation
    OdomMovement m;
    m.forward = 0.0;
    m.rightward = 0.0;
    m.angular = 0.0;
    return m;
}
