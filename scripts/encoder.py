#!/usr/bin/python3

"""
This is a module providing access to KURT's rotary encoders attached to GPIOs.

It can also be invoked directly, in which case it displays the counts of an encoder.

It does *not* work if the encoders are integrated using the device-tree overlay
and rotary-enoder driver.
"""

import argparse
import time
import sys
import pigpio


class Encoder:
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


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Read a rotary encoder connected to 2 GPIOs')
    parser.add_argument('ch_a', metavar='CH_A', type=int, nargs=1, help='Channel A GPIO (BCM numbering)')
    parser.add_argument('ch_b', metavar='CH_B', type=int, nargs=1, help='Channel B GPIO (BCM numbering)')
    parser.add_argument('--pull', type=str, help="Pull-up configuration (one of 'up', 'down', 'none'; default is 'up')")
    args = parser.parse_args()

    print(args)
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
    enc = Encoder(pi, pina, pinb, pull)

    try:
        while True:
            sys.stdout.write("Count: %d     \r" % enc.count)
            time.sleep(0.05)
    finally:
        enc.stop()
        pi.stop()
