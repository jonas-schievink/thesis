#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

class RawEncoder;

/**
 * @brief Calculates odometry messages and speed in m/s from raw encoder data.
 */
class Odometry {
    RawEncoder& m_left;
    RawEncoder& m_right;
    /// @brief Ground distance per encoder tick for each wheel.
    float m_distPerTick;
    /// @brief Medium wheel distance across their axis.
    float m_axisLength;

public:
    Odometry(RawEncoder& left, RawEncoder& right, float distPerTick, float axisLength);

    /**
     * @brief Calculates the latest value for ground speed of left wheels.
     */
    float speedLeft() const;

    /**
     * @brief Calculates the latest value for ground speed of right wheels.
     */
    float speedRight() const;

    //??? nextOdomMsg();
};

#endif // ODOMETRY_HPP
