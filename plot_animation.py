#
# Synthesis-based resolution of features/enforcers interactions in CPS
# Copyright 2020 Carnegie Mellon University.
# NO WARRANTY. THIS CARNEGIE MELLON UNIVERSITY AND SOFTWARE ENGINEERING
# INSTITUTE MATERIAL IS FURNISHED ON AN "AS-IS" BASIS. CARNEGIE MELLON
# UNIVERSITY MAKES NO WARRANTIES OF ANY KIND, EITHER EXPRESSED OR IMPLIED,
# AS TO ANY MATTER INCLUDING, BUT NOT LIMITED TO, WARRANTY OF FITNESS FOR
# PURPOSE OR MERCHANTABILITY, EXCLUSIVITY, OR RESULTS OBTAINED FROM USE OF
# THE MATERIAL. CARNEGIE MELLON UNIVERSITY DOES NOT MAKE ANY WARRANTY OF ANY
# KIND WITH RESPECT TO FREEDOM FROM PATENT, TRADEMARK, OR COPYRIGHT
# INFRINGEMENT.
# Released under a BSD (SEI)-style license, please see license.txt or contact
# permission@sei.cmu.edu for full terms.
# [DISTRIBUTION STATEMENT A] This material has been approved for public
# release and unlimited distribution.  Please see Copyright notice for
# non-US Government use and distribution.
# This Software includes and/or makes use of the following Third-Party Software
# subject to its own license:
# 1. JsonCpp
# (https://github.com/open-source-parsers/jsoncpp/blob/master/LICENSE)
# Copyright 2010 Baptiste Lepilleur and The JsonCpp Authors.
# DM20-0762
#

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

import csv
import sys

if len(sys.argv) < 2:
    print("usage: pythonw plot.py plotfile1 [plotfile2 ...]")
    sys.exit(0)

# Speed up factor (only plot every Nth element)
N = 3
        
def get_line_by_file(fname):
    """
    Create a line given a file
    """
    with open(fname, newline='') as f:
        x, y, z = [], [], []
        plot_type = f.readline().rstrip()
        reader = csv.reader(f)
        for row in reader:
            x.append(row[0])
            y.append(row[1])
            z.append(row[2])
                
    x = [float(i) for i in x]
    y = [float(i) for i in y]
    z = [-float(i) for i in z] # Need to get up position

    # Get every Nth element (for speed)
    x = x[0::N]
    y = y[0::N]
    z = z[0::N]
    
    return np.asarray([x,y,z])

def update_lines(num, dataLines, lines):
    for line, data in zip(lines, dataLines):
        # NOTE: there is no .set_data() for 3 dim data...
        line.set_data(data[0:2, :num])
        line.set_3d_properties(data[2, :num])
    return lines

# Attaching 3D axis to the figure
fig = plt.figure()
ax = p3.Axes3D(fig)

# Get lines for each data file
data = [get_line_by_file(fname) for fname in sys.argv[1:]]

# Pull the line data from each into the right plot format
lines = [ax.plot(dat[0, 0:1], dat[1, 0:1], dat[2, 0:1])[0] for dat in data]

# Setting the axes properties (based on min&max values)
ax.set_xlim3d([min([min(dat[0]) for dat in data]), max([max(dat[0]) for dat in data])])
ax.set_xlabel('X')

ax.set_ylim3d([min([min(dat[1]) for dat in data]), max([max(dat[1]) for dat in data])])
ax.set_ylabel('Y')

ax.set_zlim3d([min([min(dat[2]) for dat in data]), max([max(dat[2]) for dat in data])])
ax.set_zlabel('Z')

ax.set_title('3D Test')

# Creating the Animation object
# Blitting improves performance but makes camera rotation wonky
# Could try skipping every N element to improve speed
line_ani = animation.FuncAnimation(fig, update_lines, max([len(dat[0]) for dat in data]),
                                       fargs=(data, lines), interval=1, blit=False, repeat=False)

global pause
pause = False

def press(event):
    sys.stdout.flush()
    if event.key == ' ':
        global pause
        pause ^= True

        if(pause):
            line_ani.event_source.stop()
        else:
            line_ani.event_source.start()

fig.canvas.mpl_connect('key_press_event', press)

plt.show()
