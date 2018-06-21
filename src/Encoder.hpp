#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <ros/time.h>

/**
 * @brief Base class for rotary encoders.
 *
 * Subclasses only need to implement `read` and call the constructor.
 */
class Encoder {
    /** @brief Encoder ticks per wheel revolution */
    int m_ticksPerTurn;

    double m_revolutions;
    double m_radians;
    double m_totalRadians;
    /// @brief Time of the last update
    ros::Time m_lastUpdate;

protected:
    /**
     * @brief Creates a new encoder instance
     * @param ticksPerTurn Encoder ticks reported per wheel turn
     */
    Encoder(int ticksPerTurn);

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
     * @brief Returns the number of wheel revolutions between the last 2 @ref update calls.
     */
    double revolutions() const;

    /**
     * @brief Returns the angle the wheel has turned between the last 2 @ref update calls.
     */
    double radians() const;

    /**
     * @brief Returns the total radians the wheel has turned since the creation of this instance.
     */
    double totalRadians() const;

    /**
     * @brief Fetches and processes encoder readings, updating speed and ground distance.
     */
    void update();
};

#endif // ENCODER_HPP
