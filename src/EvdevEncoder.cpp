#include "EvdevEncoder.hpp"
#include "Utils.hpp"

#include <ros/console.h>

#include <iostream>
#include <vector>
#include <cstring>
#include <cassert>
#include <dirent.h>
#include <fcntl.h>

using std::to_string;
using std::strerror;

EvdevException::EvdevException(const std::string& what) :
    std::runtime_error(what) {}

/**
 * @brief Opens `dev_path` and matches the device name against `search_name`.
 *
 * Returns the opened device or NULL when it doesn't match the search string.
 */
static struct libevdev* matchDevice(
    const std::string& dev_path,
    const std::string& search_name)
{
    int fd = open(dev_path.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd == -1)
    {
        throw EvdevException("couldn't open() " + dev_path + ": " + strerror(errno));
    }

    struct libevdev* dev;
    int err = libevdev_new_from_fd(fd, &dev);
    if (err < 0)
    {
        throw EvdevException("couldn't open " + dev_path + " evdev: " + strerror(-err));
    }

    std::string name = libevdev_get_name(dev);

    if (name.find(search_name) != std::string::npos)
    {
        ROS_INFO("matched evdev %s with name %s against pattern %s", dev_path.c_str(), name.c_str(), search_name.c_str());
        return dev;
    }

    ROS_DEBUG("no match for evdev %s with name %s against pattern %s", dev_path.c_str(), name.c_str(), search_name.c_str());
    libevdev_free(dev);
    return NULL;
}

/**
 * @brief Prints device informations to ROS' debug stream.
 */
static void dump_device_info(struct libevdev* dev)
{
    ROS_DEBUG("evdev info of %s", libevdev_get_name(dev));
    ROS_DEBUG("unique identifier: %s", libevdev_get_uniq(dev));
    ROS_DEBUG("PID:VID:  %d:%d", libevdev_get_id_product(dev), libevdev_get_id_vendor(dev));
    ROS_DEBUG("device version: %d", libevdev_get_id_version(dev));
    ROS_DEBUG("bustype: %d", libevdev_get_id_bustype(dev));

    if (libevdev_has_event_type(dev, EV_ABS) &&
        libevdev_has_event_code(dev, EV_ABS, ABS_X))
    {
        ROS_DEBUG("has EV_ABS with ABS_X");

        ROS_DEBUG("X axis min/max: %d/%d",
            libevdev_get_abs_minimum(dev, ABS_X),
            libevdev_get_abs_maximum(dev, ABS_X));
    } else {
        ROS_WARN(
            "device does not have EV_ABS with ABS_X (encoder "
            "configuration is wrong, no data will be received)");
    }
}

EvdevEncoder::EvdevEncoder(int ticksPerTurn, const std::string& search_name, int wrap, bool invert) :
    Encoder(ticksPerTurn),
    m_wrap(wrap), m_invert(invert)
{
    // Enumerate devices in /dev/input
    std::string basedir("/dev/input");
    DIR* dir = opendir(basedir.c_str());

    if (!dir)
    {
        throw EvdevException("couldn't open " + basedir);
    }

    auto _guard = make_guard([&]() { closedir(dir); });

    std::vector<struct libevdev*> devs;
    struct dirent* ent;
    while ((ent = readdir(dir)) != NULL)
    {
        std::string filename(ent->d_name);
        if (string_starts_with(filename, "event"))
        {
            std::string path = basedir + "/" + filename;

            auto dev = matchDevice(path, search_name);
            if (dev != NULL)
            {
                devs.push_back(dev);
            }
        }
    }

    if (devs.size() == 0)
    {
        throw EvdevException("no device matching " + search_name + " found");
    }

    if (devs.size() > 1)
    {
        std::string info = "found multiple devices matching '" + search_name + "': ";
        for (auto dev : devs)
        {
            info += libevdev_get_name(dev);
            info += "; ";
            libevdev_free(dev);
        }

        throw EvdevException(info);
    }

    // exactly one device
    assert(devs.size() == 1);
    assert(devs[0] != nullptr);

    m_evdev = devs[0];
    dump_device_info(m_evdev);
    read();     // update counter
}

EvdevEncoder::~EvdevEncoder()
{
    libevdev_free(m_evdev);
}

int EvdevEncoder::read()
{
    // Drain all queued events to update the internal state
    struct input_event ev;
    while (libevdev_next_event(m_evdev, LIBEVDEV_READ_FLAG_NORMAL, &ev) != -EAGAIN);
    // (libevdev_next_event returns -EAGAIN when no events are available)

    int current = libevdev_get_event_value(m_evdev, EV_ABS, ABS_X);
    int last = m_lastCount;
    m_lastCount = current;
    //ROS_DEBUG("current axis value = %d, last = %d, wrap = %d, invert = %d", current, last, m_wrap, m_invert);

    // What theoretically is just `current - last` gets complicated due
    // to wraparound of the counter. This works in modular arithmetic. We
    // essentially have to consider 2 cases: No wraparound (the world in which
    // `current - last` is correct), and wraparound. There is need to
    // distinguish between different wrapping directions. Essentially, we "glue"
    // the [0, wrap] interval below itself, subtract `wrap` from the larger of
    // the 2 values, and re-do `current - last` with the modified values.
    // Note that, in modular arithmetic, both ways are equally valid
    // (they are said to be "congruent"). From those 2 differences, we then
    // choose the smaller one, assuming that `read` gets called frequently
    // enough to avoid extremely large readings.
    int diff1 = current - last;
    if (current > last)
    {
        current -= m_wrap;
    }
    else
    {
        last -= m_wrap;
    }
    int diff2 = current - last;
    int diff = abs(diff1) < abs(diff2) ? diff1 : diff2;
    //ROS_DEBUG("diff1 = %d, diff2 = %d, diff = %d", diff1, diff2, diff);

    return m_invert ? -diff : diff;
}
