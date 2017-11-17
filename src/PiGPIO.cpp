#include "PiGPIO.hpp"

#include <sstream>
#include <map>

namespace pigpio {

static std::string errormessage(int code)
{
    std::ostringstream s;
    s << "pigpio error " << code;
    return s.str();
}

GPIOException::GPIOException(int code) :
        runtime_error(errormessage(code)), m_code(code) {}

/**
 * @brief Checks the return value of a pigpio function and throws a
 * GPIOException if it indicates an error.
 */
static int check_retval(int retval)
{
    if (retval < 0)
    {
        throw GPIOException(retval);
    }

    return retval;
}

Pin::Pin(int number) : m_num(number)
{
    // Maybe we should internally refcount pigpio so we can deinit it?
    check_retval(gpioInitialise());
}

int Pin::number() const
{
    return m_num;
}

void Pin::output()
{
    check_retval(gpioSetMode(m_num, PI_OUTPUT));
}

void Pin::input()
{
    check_retval(gpioSetMode(m_num, PI_INPUT));
}

void Pin::pullupdown(PullUpDown config)
{
    unsigned int pud;

    switch (config)
    {
    case PullUpDown::PullUp:
        pud = PI_PUD_UP;
        break;
    case PullUpDown::PullDown:
        pud = PI_PUD_DOWN;
        break;
    case PullUpDown::None:
        pud = PI_PUD_OFF;
        break;
    }

    check_retval(gpioSetPullUpDown(m_num, pud));
}

bool Pin::digitalRead() const
{
    switch (check_retval(gpioRead(m_num)))
    {
    case PI_OFF:
        return false;
    case PI_ON:
        return true;
    case PI_TIMEOUT:
        // Watchdog timeout - Currently, watchdog support doesn't exist, so bail
        throw std::runtime_error("gpioRead reported watchdog timeout - watchdog is NYI");
    }
}

void Pin::digitalWrite(bool level)
{
    check_retval(gpioWrite(m_num, level ? PI_HIGH : PI_LOW));
}

void Pin::pwm(unsigned int duty)
{
    check_retval(gpioPWM(m_num, duty));
}

void Pin::onChange(gpioAlertFuncEx_t f, void *userdata)
{
    gpioSetAlertFuncEx(m_num, f, userdata);
}

}
