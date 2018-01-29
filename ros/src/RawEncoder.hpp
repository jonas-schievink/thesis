#ifndef RAWENCODER_HPP
#define RAWENCODER_HPP

/**
 * @brief Base class for rotary encoders.
 */
class RawEncoder {
public:
    /**
     * @brief Reads the number of counts since the last call to `read`
     *
     * If this is the first call to `read`, returns the number of counts since
     * the creation of the `Encoder`.
     */
    virtual int read() = 0;
};

#endif // RAWENCODER_HPP
