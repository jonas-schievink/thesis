#ifndef ENCODER_HPP
#define ENCODER_HPP

/**
 * @brief Base class for rotary encoders.
 *
 * Subclasses only need to implement `read` and call the constructor.
 */
class Encoder {
    /// @brief Ground distance the robot moves per encoder tick (approx.).
    double m_groundDistPerTick;

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
     * @brief Returns the ground distance covered since the last call
     */
    double readGroundDist();
};

#endif // ENCODER_HPP
