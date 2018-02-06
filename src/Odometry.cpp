#include "Odometry.hpp"
#include "Encoder.hpp"

#include <tf/transform_broadcaster.h>
#include <cmath>

Odometry::Odometry(ros::NodeHandle node, Encoder& left, Encoder& right, float axisLength) :
    m_node(node), m_left(left), m_right(right), m_axisLength(axisLength),
    m_x(0.0), m_y(0.0), m_theta(0.0)
{
    m_odomPub = m_node.advertise<nav_msgs::Odometry>("/odom", 1);
    m_jointPub = m_node.advertise<sensor_msgs::JointState>("/joint_states", 1);
}

void Odometry::update()
{
    /*nav_msgs::Odometry odom;
    calcMovement(&odom);
    m_odomPub.publish(odom);*/

    sensor_msgs::JointState joints;
    calcJointState(&joints);
    m_jointPub.publish(joints);
}

void Odometry::calcJointState(sensor_msgs::JointState* msg)
{
    m_leftWheel += m_left.revolutions() * 2.0 * M_PI;
    m_rightWheel += m_right.revolutions() * 2.0 * M_PI;

    if (m_leftWheel < -M_PI) m_leftWheel += 2.0 * M_PI;
    if (m_leftWheel >  M_PI) m_leftWheel -= 2.0 * M_PI;
    if (m_rightWheel < -M_PI) m_rightWheel += 2.0 * M_PI;
    if (m_rightWheel >  M_PI) m_rightWheel -= 2.0 * M_PI;

    msg->header.stamp = ros::Time::now();
    msg->name.resize(6);
    msg->position.resize(6);
    msg->name[0] = "left_front_wheel_joint";
    msg->name[1] = "left_middle_wheel_joint";
    msg->name[2] = "left_rear_wheel_joint";
    msg->name[3] = "right_front_wheel_joint";
    msg->name[4] = "right_middle_wheel_joint";
    msg->name[5] = "right_rear_wheel_joint";
    msg->position[0] = m_leftWheel;
    msg->position[1] = m_leftWheel;
    msg->position[2] = m_leftWheel;
    msg->position[3] = m_rightWheel;
    msg->position[4] = m_rightWheel;
    msg->position[5] = m_rightWheel;
}

void Odometry::calcMovement(nav_msgs::Odometry* msg)
{
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = "odom_combined";
    // `child_frame_id` isn't documented, so copy what the old node was doing.
    msg->child_frame_id = "base_footprint";

    // We need to obtain `pose` and `twist` data. `pose` is the robot position
    // in its own reference frame (accumulated since ROS was started, basically)
    // while `twist` is the current linear and angular velocity.
    // (of course, of ROS were any good, we'd only have to supply `twist` and
    // the system integrates the `pose` automatically, but that ain't happening)

    // We can easily calculate the forward velocity:
    double vLeft = m_left.speed();
    double vRight = m_right.speed();
    double velocity = (vLeft + vRight) / 2.0;   // average of l/r vel
    // Deriving angular vel/pos is somewhat complex, refer to some ext. source
    double angularVelocity = (vRight - vLeft) / m_axisLength; // in rad/s

    // The `twist` is in the robot's local reference frame.
    // x = forward, y = left (= 0)
    msg->twist.twist.linear.x = velocity;
    msg->twist.twist.linear.y = 0;
    // Rotation around z = up axis
    msg->twist.twist.angular.z = angularVelocity;


    // The change in position since the last update works similarly:
    double deltaLeft = m_left.groundDist();
    double deltaRight = m_right.groundDist();
    double delta = (deltaLeft + deltaRight) / 2.0;   // average of l/r dist
    double deltaAngle = (deltaRight - deltaLeft) / m_axisLength; // in rad

    // Now integrate the global position/angle. Also refer to an external source
    // for an explanation of this.
    m_x += delta * cos(m_theta + deltaAngle / 2.0);
    m_y += delta * sin(m_theta + deltaAngle / 2.0);
    m_theta += deltaAngle;

    msg->pose.pose.position.x = m_x;
    msg->pose.pose.position.y = -m_y;   // swap left and right
    msg->pose.pose.position.y = 0.0;
    msg->pose.pose.orientation = tf::createQuaternionMsgFromYaw(-m_theta);
}
