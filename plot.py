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

from mpl_toolkits.mplot3d import axes3d  # noqa: F401 unused import
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import csv
import sys

if len(sys.argv) < 2:
    print("usage: pythonw plot.py plotfile1 [plotfile2 ...]")
    sys.exit(0)

fig = plt.figure()
ax = fig.gca(projection='3d')

for fname in sys.argv[1:]:
    x, y, z = [], [], []
    with open(fname, newline='') as f:
        plot_type = f.readline().rstrip()
        reader = csv.reader(f)
        for row in reader:
            x.append(row[0])
            y.append(row[1])
            z.append(row[2])

    x = [float(i) for i in x]
    y = [float(i) for i in y]
    z = [int(i)   for i in z]

    if plot_type == 'scatter':
        ax.scatter(x, y, z, zdir='z')
    else:
        ax.plot(x, y, z, zdir='z')
        
# Make legend, set axes limits and labels
# ax.legend(loc='upper left')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Time')

plt.show()
