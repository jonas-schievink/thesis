#ifndef EVDEVENCODER_HPP
#define EVDEVENCODER_HPP

#include "Encoder.hpp"

#include <libevdev/libevdev.h>

#include <string>
#include <stdexcept>

/**
 * @brief Exception thrown when an `EvdevEncoder` can not find a matching input
 * device.
 */
class EvdevNameException : public std::runtime_error {
public:
    EvdevNameException(const std::string& what);
};

/**
 * @brief A rotary encoder evdev
 *
 * This can be used to attach a Linux rotary-encoder device. The device must be
 * configured to report events using the ABS_X axis (absolute axis).
 */
class EvdevEncoder : public Encoder {
    struct libevdev* m_evdev;

public:
    /**
     * @brief Create a new evdev encoder by searching for an input device
     * @param search_name Device name the encoder has to match
     *
     * This will search all `/dev/input/event*` devices for an input device
     * whose name matches `search_name`.
     *
     * To figure out the device name, the `evtest` tool can be used.
     *
     * If no or more than 1 device matches the name, an `EvdevNameException`
     * will be thrown.
     */
    EvdevEncoder(const std::string& search_name);

    ~EvdevEncoder();

    int read() override;
};

#endif // EVDEVENCODER_HPP
