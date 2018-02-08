#!/usr/bin/python3

"""
This is a module providing access to KURT's rotary encoders attached to GPIOs or
via evdev (see kernel directory).

It can also be invoked directly, in which case it displays the counts of an
encoder.
"""

import argparse
import time
import sys
import pigpio
from evdev import InputDevice, ecodes
from threading import Thread


class GPIOEncoder:
    """
    Counts ticks of a rotary encoder.
    """

    _lvl_a = 0
    _lvl_b = 0
    count = 0

    def __init__(self, pi, ch_a, ch_b, pull=None):
        """
        Creates a new Encoder instance using the pins `ch_a` and `ch_b`.

        `pi` is an instance of `pigpio.pi`, `ch_a` and `ch_b` the BCM pin numbers of the channels,
        `pull` the pullup configuration (any `pigpio.PUD_*` constant). If `pull` is omitted or `None`,
        pull-ups are enabled.

        This function will also start counting encoder ticks immediately.
        """

        if pull is None:
            pull = pigpio.PUD_UP
        pi.set_mode(ch_a, pigpio.INPUT)
        pi.set_mode(ch_b, pigpio.INPUT)
        pi.set_pull_up_down(ch_a, pull)
        pi.set_pull_up_down(ch_b, pull)
        self._ch_a = ch_a
        self._ch_b = ch_b
        self._lastgpio = ch_a

        def callback(gpio, level, tick):
            """
            Called when a channel changes levels
            """

            # Code adjusted from C rotary encoder example
            if gpio == self._ch_a:
                self._lvl_a = level
            else:
                self._lvl_b = level

            if self._lastgpio != gpio:
                self._lastgpio = gpio

                if gpio == self._ch_a and level == 1:
                    if self._lvl_b:
                        self.count += 1
                elif gpio == self._ch_b and level == 1:
                    if self._lvl_a:
                        self.count -= 1

        self._cb_a = pi.callback(ch_a, pigpio.EITHER_EDGE, callback)
        self._cb_b = pi.callback(ch_b, pigpio.EITHER_EDGE, callback)

    def stop(self):
        """
        Disables the level change callbacks, stops counting ticks.
        """
        self._cb_a.cancel()
        self._cb_b.cancel()


class EvdevEncoder:
    count = 0
    _wrap = None  # number of reported counts is mod this
    _dev = None
    _thread = None
    _rawcount = None

    def __init__(self, dev):
        self._dev = InputDevice(dev)

        events = self._dev.capabilities(absinfo=True)
        if ecodes.EV_ABS not in events:
            raise IOError("encoder device doesn't support EV_ABS events")
        absinfo = events[ecodes.EV_ABS][0][1]
        if absinfo.min != 0:
            raise IOError("axis min value should be 0, is %d" % absinfo.min)

        self._wrap = absinfo.max

        self._thread = Thread(target=EvdevEncoder.eventThread, args=(self,))
        self._thread.daemon = True
        self._thread.start()

    def eventThread(self):
        for event in self._dev.read_loop():
            if event.type == ecodes.EV_ABS and event.code == ecodes.ABS_X:
                if self._rawcount is None:
                    self._rawcount = event.value
                    continue

                current = event.value
                last = self._rawcount
                self._rawcount = current

                diff1 = current - last
                if current > last:
                    current -= self._wrap
                else:
                    last -= self._wrap
                diff2 = current - last
                if abs(diff1) < abs(diff2):
                    diff = diff1
                else:
                    diff = diff2
                self.count += diff

    def stop(self):
        # FIXME: Close the thread
        pass


if __name__ == "__main__":
    def cmd_gpio(args):
        pina = args.ch_a[0]
        pinb = args.ch_b[0]
        pull = args.pull

        if pull is None:
            # Pull-ups enabled by default
            pull = 'up'

        if pull == 'up':
            pull = pigpio.PUD_UP
        elif pull == 'down':
            pull = pigpio.PUD_DOWN
        elif pull == 'none':
            pull = pigpio.PUD_NONE
        else:
            print("invalid value for '--pull': %s (expected one of 'up', 'down', 'none')" % pull)
            exit(1)

        pi = pigpio.pi()
        enc = GPIOEncoder(pi, pina, pinb, pull)

        try:
            while True:
                sys.stdout.write("Count: %d     \r" % enc.count)
                time.sleep(0.05)
        finally:
            enc.stop()
            pi.stop()

    def cmd_evdev(args):
        dev = args.dev[0]

        enc = EvdevEncoder(dev)
        while True:
                sys.stdout.write("Count: %d     \r" % enc.count)
                time.sleep(0.05)


    parser = argparse.ArgumentParser(description='Read a rotary encoder')
    subparsers = parser.add_subparsers(dest='subcommand')
    subparsers.required = True

    gpio = subparsers.add_parser('gpio')
    gpio.set_defaults(func=cmd_gpio)
    gpio.add_argument('ch_a', metavar='CH_A', type=int, nargs=1, help='Channel A GPIO (BCM numbering)')
    gpio.add_argument('ch_b', metavar='CH_B', type=int, nargs=1, help='Channel B GPIO (BCM numbering)')
    gpio.add_argument('--pull', type=str, help="Pull-up configuration (one of 'up', 'down', 'none'; default is 'up')")

    evdev = subparsers.add_parser('evdev')
    evdev.set_defaults(func=cmd_evdev)
    evdev.add_argument('dev', metavar='DEV', type=str, nargs=1, help='Path to device file (eg. "/dev/input/event0")')

    args = parser.parse_args()
    print(args)
    args.func(args)
