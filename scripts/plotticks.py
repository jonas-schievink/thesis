#!/usr/bin/python3

"""
Ramps up a motor's PWM signal and plots the resulting counts per second reported by the encoder.

Data is output in CSV format.
"""

import argparse
import sys
import time
from encoder import GPIOEncoder, EvdevEncoder
from motor import Motor
import pigpio

def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

if __name__ == "__main__":
    pi = None

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
            eprint("invalid value for '--pull': %s (expected one of 'up', 'down', 'none')" % pull)
            exit(1)

        eprint("Configuring encoder...")
        return GPIOEncoder(pi, pina, pinb, pull)

    def cmd_evdev(args):
        dev = args.dev[0]
        return EvdevEncoder(dev)


    parser = argparse.ArgumentParser(description='Plots encoder counts per second while increasing motor speed')
    parser.add_argument('enablepin', metavar='ENABLEPIN', type=int, nargs=1, help='Pin enabling the motor (BCM numbering, driven with PWM)')
    parser.add_argument('dirpin', metavar='DIRPIN', type=int, nargs=1, help='Pin controlling the motor direction (BCM numbering)')
    parser.add_argument('--freq', type=int, help='PWM frequency for enable pin (default: 100 Hz)')
    parser.add_argument('--steps', type=int, help='Speed steps to measure (default: 1000)')
    parser.add_argument('--sleep', type=int, help='Milliseconds to wait for motor to reach speed, and time to measure encoder ticks (default: 100)')
    subparsers = parser.add_subparsers(dest='subcommand')
    subparsers.required = True

    gpio = subparsers.add_parser('gpio')
    gpio.set_defaults(func=cmd_gpio)
    gpio.add_argument('ch_a', metavar='CH_A', type=int, nargs=1, help='Encoder channel A GPIO (BCM numbering)')
    gpio.add_argument('ch_b', metavar='CH_B', type=int, nargs=1, help='Encoder channel B GPIO (BCM numbering)')
    gpio.add_argument('--pull', type=str, help="Pull-up configuration (one of 'up', 'down', 'none'; default is 'up')", choices=['up', 'down', 'none'])

    evdev = subparsers.add_parser('evdev')
    evdev.set_defaults(func=cmd_evdev)
    evdev.add_argument('dev', metavar='DEV', type=str, nargs=1, help='Path to encoder device (eg. "/dev/input/event0")')

    args = parser.parse_args()
    eprint(args)

    eprint("Connecting to pigpiod...")
    pi = pigpio.pi()

    encoder = args.func(args)
    enablepin = args.enablepin[0]
    dirpin = args.dirpin[0]
    steps = args.steps
    sleep = args.sleep
    freq = args.freq

    if steps is None:
        steps = 1000

    if sleep is None:
        sleep = 100
    sleep /= 1000

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


    eprint("")
    print("Motor speed (-1..1),Encoder ticks per Second")
    try:
        for step in range(steps):
            measurestep(step)

        motor.setspeed(0.0)
        time.sleep(1.0)

        for step in range(0, -steps):
            measurestep(step)
        motor.setspeed(0)
    finally:
        motor.setspeed(0)
        encoder.stop()
        pi.stop()
