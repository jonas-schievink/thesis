#!/usr/bin/env python2

import matplotlib.pyplot as plt

minx = 0.0
maxx = 0.0
miny = 0.0
maxy = 0.0

fodom = open('plot-odom.csv', 'r')
odomx = []
odomy = []
for line in fodom.readlines():
    xy = line.split(',')
    x = float(xy[0])
    y = float(xy[1])
    minx = min(minx, x)
    maxx = max(maxx, x)
    miny = min(miny, y)
    maxy = max(maxy, y)
    odomx.append(x)
    odomy.append(y)

fcmdvel = open('plot-cmdvel.csv', 'r')
cmdvelx = []
cmdvely = []
for line in fcmdvel.readlines():
    xy = line.split(',')
    x = float(xy[0])
    y = float(xy[1])
    minx = min(minx, x)
    maxx = max(maxx, x)
    miny = min(miny, y)
    maxy = max(maxy, y)
    cmdvelx.append(x)
    cmdvely.append(y)

plt.axis([minx, maxx+0.01, miny, maxy+0.01])
cmdvel = plt.plot(cmdvelx, cmdvely, color='blue')
odom = plt.plot(odomx, odomy, color='red')
plt.legend(['Zielgeschwindigkeit (/cmd_vel)', 'Wirkliche Geschwindigkeit (/odom)'])
plt.show()
