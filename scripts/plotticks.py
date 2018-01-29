#!/usr/bin/python3

"""
Ramps up a motor's PWM signal and plots the resulting counts per second reported by the encoder.

Data is output in CSV format.
"""

import argparse
import sys
import time
from encoder import Encoder
from motor import Motor
import pigpio

def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plots encoder counts per second while increasing motor speed')
    parser.add_argument('enablepin', metavar='ENABLEPIN', type=int, nargs=1, help='Pin enabling the motor (BCM numbering, driven with PWM)')
    parser.add_argument('dirpin', metavar='DIRPIN', type=int, nargs=1, help='Pin controlling the motor direction (BCM numbering)')
    parser.add_argument('ch_a', metavar='CH_A', type=int, nargs=1, help='Encoder channel A GPIO (BCM numbering)')
    parser.add_argument('ch_b', metavar='CH_B', type=int, nargs=1, help='Encoder channel B GPIO (BCM numbering)')
    parser.add_argument('--pull', type=str, help="Pull-up configuration (one of 'up', 'down', 'none'; default is 'up')")
    parser.add_argument('--freq', type=int, help='PWM frequency for enable pin (default: 100 Hz)')
    parser.add_argument('--steps', type=int, help='Speed steps to measure (default: 1000)')
    parser.add_argument('--sleep', type=int, help='Milliseconds to wait for motor to reach speed, and time to measure encoder ticks (default: 100)')
    args = parser.parse_args()

    eprint(args)
    enablepin = args.enablepin[0]
    dirpin = args.dirpin[0]
    pina = args.ch_a[0]
    pinb = args.ch_b[0]
    pull = args.pull
    freq = args.freq
    steps = args.steps
    sleep = args.sleep

    if steps is None:
        steps = 1000

    if sleep is None:
        sleep = 100
    sleep /= 1000

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
        eprint("invalid value for '--pull': %s (expected one of 'up', 'down', 'none')" % pull)
        exit(1)

    eprint("Connecting to pigpiod...")
    pi = pigpio.pi()
    eprint("Configuring encoder...")
    encoder = Encoder(pi, pina, pinb, pull)
    eprint("Configuring motor...")
    motor = Motor(pi, enablepin, dirpin, freq)

    eprint("Estimated time needed: {} seconds".format(steps * 2 * sleep))

    def measurestep(step):
        spd = step/steps
        eprint("step {}/{} spd {}".format(step, steps, spd))
        motor.setspeed(spd)
        time.sleep(sleep)
        encoder.count = 0
        time.sleep(sleep)
        tps = encoder.count / sleep
        print("{},{}".format(spd, tps))     # CSV output


    try:
        # FIXME: Better: range(-steps, steps)?
        for step in range(steps):
            measurestep(step)
        for step in range(0, -steps):
            measurestep(step)
        motor.setspeed(0)
    finally:
        encoder.stop()
        pi.stop()
