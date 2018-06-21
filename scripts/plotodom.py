#!/usr/bin/env python2

"""
Plots the reported robot speed on /odom while ramping motor speed up or down
on /cmd_vel.

NOTE: This is a Python 2 script, the others are Python 3! (Thanks, ROS!)
"""

import argparse
from sys import stderr
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def calcCurve(curve, t):
    """
    Calculates the velocity value according to a specified curve type.
    `t` is the point along the curve, from 0 to 1 inclusive.
    The returned value is in range 0 to 1 (inclusive) and should be scaled to
    the desired velocity range.
    """
    if curve == 'none':
        return 0.0
    elif curve == 'const':
        return 1.0
    elif curve == 'rampupdown':
        # ramp up to 1 until t=0.5, then back down to 0
        if t < 0.5:
            return t*2
        else:
            return 1.0 - (t-0.5)*2
    elif curve == 'onoff':
        if t < 0.5:
            return 1.0
        else:
            return 0.0

def buildTwist(vel):
    """
    Build a Twist message to send to /cmd_vel from a velocity
    """
    msg = Twist()
    msg.linear.x = vel
    return msg

def odomCallback(odom):
    print 'odom',odom.header.stamp,odom.twist.twist.linear.x

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plots reported /odom speed while changing motor speeds via /cmd_vel')
    parser.add_argument('--time', type=float, default=3, help='Time to spend in each repetition (seconds)')
    parser.add_argument('--repeat', type=int, default=1, help='Number of repetitions')
    parser.add_argument('--max-vel', type=float, default=0.1, help='Max. velocity to request')
    parser.add_argument('--step-delay', type=float, default=0.05, help='Delay between /cmd_vel updates')
    parser.add_argument('curve', type=str, help='Kind of velocity curve to send', choices=[
        'none','const','rampupdown','onoff'
    ])

    args = parser.parse_args()
    curve = args.curve

    rospy.init_node('odom_plotter', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/odom', Odometry, odomCallback)

    print 'repetitions,', args.repeat
    print 'seconds,', args.time
    print 'curve,', curve
    print 'max velocity,', args.max_vel
    print 'step delay,', args.step_delay

    for i in range(args.repeat):
        starttime = time.time()
        endtime = starttime + args.time
        now = starttime

        # we guarantee that the first value of t is 0 and the last is 1.
        # we do not guarantee any defined number of /cmd_vel updates, only the
        # approximate time between them.
        while True:
            if now >= endtime:
                t = 1.0
            elif now < starttime:
                # just to make sure
                print >> stderr, "Great Scott! Time just went backwards!"
                print >> stderr, "start=%f,end=%f,now=%f" % (starttime,endtime,now)
                sys.exit(1)
            else:
                # calculate how far along we are (0..=1)
                t = (now - starttime) / (endtime - starttime)

            # publish update
            vel = calcCurve(curve, t)
            print 'cmd_vel',now,vel
            pub.publish(buildTwist(vel))

            if t == 1.0:
                break

            now = time.time()
            time.sleep(0.01)

    # stop motors
    pub.publish(buildTwist(0.0))
