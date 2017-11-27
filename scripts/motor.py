#!/usr/bin/python3

import curses
import argparse
import pigpio

DEFAULT_PWM_FREQ = 100
DEFAULT_PWM_RANGE = 1024

class Motor:
    """
    Allows to control a motor connected to 2 GPIOs.
    """

    def __init__(self, pi, enablepin, dirpin, freq=None):
        """
        Creates a new Motor instance.

        `pi` is the instance of `pigpio.pi` to use.
        `enablepin` is the GPIO that drives the motor. It will be driven with PWM at a frequency of
        `freq` (or a default frequency if `freq` is None).
        `dirpin` is a GPIO that inverts the direction the motor is moving in.
        """
        self._pi = pi
        self._enablepin = enablepin
        self._dirpin = dirpin
        if freq is None:
            freq = DEFAULT_PWM_FREQ
        pi.set_PWM_frequency(enablepin, freq)
        pi.set_PWM_dutycycle(enablepin, 0)
        pi.set_PWM_range(enablepin, DEFAULT_PWM_RANGE)
        pi.write(dirpin, 0)     # forward
        print("GPIOs configured. Real PWM resolution: %d" % pi.get_PWM_real_range(enablepin))

    def setspeed(self, spd):
        """
        Sets the speed (and direction) of the motor.

        `spd` is a number between -1.0 and 1.0 (inclusive). If `spd` is negative, the direction of
        the motor will be inverted.
        """
        if spd < 0:
            dir = 1     # reverse
            spd = abs(spd)
        else:
            dir = 0     # forward
        self._pi.write(self._dirpin, dir)
        self._pi.set_PWM_dutycycle(self._enablepin, spd * DEFAULT_PWM_RANGE)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Control a motor on the GPIOs with the keyboard')
    parser.add_argument('enablepin', metavar='ENABLEPIN', type=int, nargs=1, help='Pin enabling the motor (BCM numbering, driven with PWM)')
    parser.add_argument('dirpin', metavar='DIRPIN', type=int, nargs=1, help='Pin controlling the motor direction (BCM numbering)')
    parser.add_argument('--freq', type=int, help='PWM frequency for enable pin (default: 100 Hz)')
    args = parser.parse_args()

    print(args)
    enablepin = args.enablepin[0]
    dirpin = args.dirpin[0]
    freq = args.freq

    pins = pigpio.pi()
    motor = Motor(pins, enablepin, dirpin, freq)

    # integer, goes from -SPEED_MAX to SPEED_MAX
    speed = 0
    SPEED_MAX = 1000
    step_choice = 0     # index into STEPS
    STEPS = [1, 5, 10, 50, 100, 1000]

    def main(win):
        global speed, step_choice
        while True:
            win.erase()
            win.addstr(0, 0, "Speed: %d / %d" % (speed, SPEED_MAX))
            win.addstr(2, 0, "Step: %d" % STEPS[step_choice])
            win.refresh()

            key = win.getch()
            if key == curses.KEY_UP:
                speed += STEPS[step_choice]
            elif key == curses.KEY_DOWN:
                speed -= STEPS[step_choice]
            elif key == curses.KEY_LEFT:
                step_choice -= 1
            elif key == curses.KEY_RIGHT:
                step_choice += 1
            elif key == ord('0'):
                speed = 0

            speed = max(speed, -SPEED_MAX)
            speed = min(speed, SPEED_MAX)
            step_choice = max(step_choice, 0)
            step_choice = min(step_choice, len(STEPS))
            motor.setspeed(speed / SPEED_MAX)

    try:
        curses.wrapper(main)
    finally:
        motor.setspeed(0.0)
        pins.stop()
