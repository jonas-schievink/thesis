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

EvdevNameException::EvdevNameException(const std::string& what) :
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
    int fd = open(dev_path.c_str(), O_RDONLY);
    if (fd == -1)
    {
        throw EvdevNameException("couldn't open() " + dev_path + ": " + strerror(errno));
    }

    struct libevdev* dev;
    int err = libevdev_new_from_fd(fd, &dev);
    if (err < 0)
    {
        throw EvdevNameException("couldn't open " + dev_path + " evdev: " + strerror(-err));
    }

    std::string name = libevdev_get_name(dev);

    if (name.find(search_name) != std::string::npos)
    {
        ROS_DEBUG("matched evdev %s with name %s against pattern %s", dev_path.c_str(), name.c_str(), search_name.c_str());
        return dev;
    }

    ROS_DEBUG("no match for evdev %s with name %s against pattern %s", dev_path.c_str(), name.c_str(), search_name.c_str());
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

EvdevEncoder::EvdevEncoder(const std::string& search_name)
{
    // Enumerate devices in /dev/input
    std::string basedir("/dev/input");
    DIR* dir = opendir(basedir.c_str());

    if (!dir)
    {
        throw EvdevNameException("couldn't open " + basedir);
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
        throw EvdevNameException("no device matching " + search_name + " found");
    }

    if (devs.size() > 1)
    {
        std::string info = "found multiple devices matching '" + search_name + "': ";
        for (auto dev : devs)
        {
            info += libevdev_get_name(dev);
            info += "; ";
        }

        throw EvdevNameException(info);
    }

    // exactly one device
    assert(devs.size() == 1);
    assert(devs[0] != nullptr);

    m_evdev = devs[0];
    dump_device_info(m_evdev);
}

EvdevEncoder::~EvdevEncoder()
{
    libevdev_free(m_evdev);
}

int EvdevEncoder::read()
{
    return -1;
}
