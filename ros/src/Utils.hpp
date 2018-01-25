#ifndef UTILS_HPP
#define UTILS_HPP

#include <string>
#include <functional>

/** garbage language */
bool string_starts_with(const std::string& s, const std::string& prefix);

/**
 * @brief Calls a function in its destructor (for makeshift RAII)
 */
class Guard {
    std::function<void()> m_func;

public:
    Guard(std::function<void()> what) : m_func(what) {}

    ~Guard() { m_func(); }
};

Guard make_guard(std::function<void()> what);

#endif // UTILS_HPP
