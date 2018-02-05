#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <ros/time.h>

/**
 * @brief Base class for rotary encoders.
 *
 * Subclasses only need to implement `read` and call the constructor.
 */
class Encoder {
    int m_ticksPerTurn;
    /// @brief Ground distance the robot moves per encoder tick (approx.).
    double m_groundDistPerTick;

    double m_revolutions;
    double m_groundDist;
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
    double groundDist() const;

    /**
     * @brief Returns the average speed in m/s since the last @ref update call.
     */
    double speed() const;

    /**
     * @brief Returns the number of wheel revolutions since last @ref update call.
     */
    double revolutions() const;

    /**
     * @brief Fetches and processes encoder readings, updating speed and ground distance.
     */
    void update();
};

#endif // ENCODER_HPP
