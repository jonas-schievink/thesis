#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

class Encoder;

/**
 * @brief Calculates odometry messages and speed in m/s from raw encoder data.
 *
 * Publishes odometry data and allows the speed controller to query the current
 * motor (ground) speed.
 */
class Odometry {
    Encoder& m_left;
    Encoder& m_right;
    /// @brief Encoder ticks per wheel turn.
    int m_ticksPerTurn;
    /// @brief Wheel perimeter in meters.
    float m_wheelPerimeter;
    /// @brief Medium wheel distance across their axis in meters.
    float m_axisLength;

public:
    Odometry(Encoder& left, Encoder& right, int ticksPerTurn, float wheelPerimeter, float axisLength);

    /**
     * @brief Calculates the latest value for ground speed of left wheels.
     */
    float speedLeft() const;

    /**
     * @brief Calculates the latest value for ground speed of right wheels.
     */
    float speedRight() const;

    /**
     * @brief Update odometry state by reading from the encoders.
     */
    void update();
    //??? nextOdomMsg();
};

#endif // ODOMETRY_HPP
