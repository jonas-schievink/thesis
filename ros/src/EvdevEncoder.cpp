#include "EvdevEncoder.hpp"
#include "Utils.hpp"

#include <iostream>
#include <vector>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>

using std::cout;
using std::endl;
using std::to_string;
using std::strerror;

EvdevNameException::EvdevNameException(const std::string& what) :
    std::runtime_error(what) {}

/**
 * Opens `dev_path` and matches the device name against `search_name`.
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
    cout << "device " << dev_path << " name is " << name;

    if (name.find(search_name) != std::string::npos)
    {
        cout << " -> match" << endl;
        return dev;
    }

    cout << " -> no match" << endl;
    return NULL;
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
    m_evdev = devs[0];
}

int EvdevEncoder::read()
{
    return -1;
}
