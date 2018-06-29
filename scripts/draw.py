#!/usr/bin/env python2

import argparse
import matplotlib.pyplot as plt
from matplotlib import rc

parser = argparse.ArgumentParser(description='Draw graphs from CSV files produced by plotodom.py')
parser.add_argument('data_dirs', type=str, help='Directories containing the plot-odom.csv and plot-cmdvel.csv files', nargs='+')
parser.add_argument('--out', type=str, help='Output file (can be any image file supported by matplotlib, but is usually pgf)')
args = parser.parse_args()

minx = 0.0
maxx = 0.0
miny = 0.0
maxy = 0.0

def read(d):
    global minx, maxx, miny, maxy

    fodom = open(d + '/plot-odom.csv', 'r')
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

    fcmdvel = open(d + '/plot-cmdvel.csv', 'r')
    cmdvelx = [-0.001]
    cmdvely = [0]
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

    return (odomx, odomy, cmdvelx, cmdvely)


# configure TeX
rc('text', usetex=True)
rc('pgf', rcfonts=False)

# we need to determine the subplot layout (rows and columns) from the dir count given
# use at most 2 columns of graphs:
cols = 1
if len(args.data_dirs) > 1:
    cols = 2
if len(args.data_dirs) == 1:
    rows = 1
else:
    rows = (len(args.data_dirs) + 1) / cols   # adding 1 to round up. this might leave a free space.

print "Creating %d graphs, %d rows by %d cols" % (len(args.data_dirs), rows, cols)

# set up global graph options
fig, subplots = plt.subplots(nrows=rows, ncols=cols, sharex=True, sharey=True, figsize=(7.75,6.5), squeeze=False)
subplots = subplots.reshape(cols * rows)

# in case we want to set the axis range ourselves:
# (unused, matplotlib does a good job with this as long as we use subplots and do it in one pass)
#plt.axis([minx, maxx+0.01, miny, maxy+0.01])


# create one subplot for each specified dir
for idx, d in enumerate(args.data_dirs):
    print "Processing dir '" + d + "'..."
    (odomx, odomy, cmdvelx, cmdvely) = read(d)

    plot = subplots[idx]
    plot.plot(cmdvelx, cmdvely, color='blue')
    plot.plot(odomx, odomy, color='red')

    if idx / cols == rows - 1:
        # last row, add bottom labels
        plot.set_xlabel('Zeit (s)')
    if idx % 2 == 0:
        # left column, add left labels
        plot.set_ylabel('Geschwindigkeit ($m \over s$)')
    if idx == 1:
        plot.legend([r'Zielgeschwindigkeit (\texttt{/cmd\_vel})', r'Wirkliche Geschwindigkeit (\texttt{/odom})'])


if args.out is None:
    plt.show()
else:
    print "Saving as " + args.out
    plt.savefig(args.out)
