#!/bin/bash
##
## Synthesis-based resolution of features/enforcers interactions in CPS
## Copyright 2020 Carnegie Mellon University.
## NO WARRANTY. THIS CARNEGIE MELLON UNIVERSITY AND SOFTWARE ENGINEERING
## INSTITUTE MATERIAL IS FURNISHED ON AN "AS-IS" BASIS. CARNEGIE MELLON
## UNIVERSITY MAKES NO WARRANTIES OF ANY KIND, EITHER EXPRESSED OR IMPLIED,
## AS TO ANY MATTER INCLUDING, BUT NOT LIMITED TO, WARRANTY OF FITNESS FOR
## PURPOSE OR MERCHANTABILITY, EXCLUSIVITY, OR RESULTS OBTAINED FROM USE OF
## THE MATERIAL. CARNEGIE MELLON UNIVERSITY DOES NOT MAKE ANY WARRANTY OF ANY
## KIND WITH RESPECT TO FREEDOM FROM PATENT, TRADEMARK, OR COPYRIGHT
## INFRINGEMENT.
## Released under a BSD (SEI)-style license, please see license.txt or contact
## permission@sei.cmu.edu for full terms.
## [DISTRIBUTION STATEMENT A] This material has been approved for public
## release and unlimited distribution.  Please see Copyright notice for
## non-US Government use and distribution.
## This Software includes and/or makes use of the following Third-Party Software
## subject to its own license:
## 1. JsonCpp
## (https://github.com/open-source-parsers/jsoncpp/blob/master/LICENSE)
## Copyright 2010 Baptiste Lepilleur and The JsonCpp Authors.
## DM20-0762
##

if [ "$1" == "" ]; then
    echo "Usage: save_splots.sh <name>"
else
   
mkdir -p ./plots/$1

cp ./plot_my_drone.dat plots/$1
cp ./plot_enemy_drone.dat plots/$1
cp ./plot_chase_points.dat plots/$1
cp ./plot_coordinated_points.dat plots/$1
cp ./drone.cfg plots/$1
cp ./plot_animation_data_ego.dat plots/$1
cp ./plot_animation_data_enemy.dat plots/$1
cp ./test.log plots/$1

echo "Plots saved to ./plots/${1}"
fi
