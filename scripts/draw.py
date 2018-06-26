#!/usr/bin/env python2

import matplotlib.pyplot as plt

fodom = open('plot-odom.csv', 'r')
odomx = []
odomy = []
for line in fodom.readlines():
    xy = line.split(',')
    odomx.append(xy[0])
    odomy.append(xy[1])

fcmdvel = open('plot-cmdvel.csv', 'r')
cmdvelx = []
cmdvely = []
for line in fcmdvel.readlines():
    xy = line.split(',')
    cmdvelx.append(xy[0])
    cmdvely.append(xy[1])

#plt.axis(xmin=0, ymin=0)
cmdvel = plt.plot(cmdvelx, cmdvely, color='blue')
odom = plt.plot(odomx, odomy, color='red')
plt.legend(['Zielgeschwindigkeit (/cmd_vel)', 'Wirkliche Geschwindigkeit (/odom)'])
plt.show()
