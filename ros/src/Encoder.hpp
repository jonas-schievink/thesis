#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <ros/time.h>

/**
 * @brief Base class for rotary encoders.
 *
 * Subclasses only need to implement `read` and call the constructor.
 */
class Encoder {
    /// @brief Ground distance the robot moves per encoder tick (approx.).
    double m_groundDistPerTick;

    /// @brief Ground distance moved between the last 2 calls to @ref update.
    double m_groundDist;
    /// @brief Average speed between the last 2 calls to @ref update.
    double m_speed;
    /// @brief Time of the last update
    ros::Time m_lastUpdate;

protected:
    /**
     * @brief Creates a new encoder instance
     * @param ticksPerTurn Encoder ticks reported per wheel turn
     * @param wheelPerimeter Wheel perimeter in meters
     */
    Encoder(int ticksPerTurn, float wheelPerimeter);

    Encoder(const Encoder& other) = delete;
    Encoder& operator=(const Encoder& other) = delete;

    /**
     * @brief Reads the number of counts since the last call to `read`
     *
     * If this is the first call to `read`, returns the number of counts since
     * the creation of this instance.
     */
    virtual int read() = 0;

public:
    /**
     * @brief Returns the ground distance covered since the last @ref update call.
     */
    double groundDist();

    /**
     * @brief Returns the average speed in m/s since the last @ref update call.
     */
    double speed();

    /**
     * @brief Fetches and processes encoder readings, updating speed and ground distance.
     */
    void update();
};

#endif // ENCODER_HPP
