#include "Utils.hpp"

bool string_starts_with(const std::string& s, const std::string& prefix)
{
    if (s.compare(0, prefix.size(), prefix) == 0)
    {
        return true;
    }

    return false;
}

Guard make_guard(std::function<void()> what)
{
    return Guard(what);
}
