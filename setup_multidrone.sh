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

# Don't leave anything dangling if script gets SIGINT'd
trap "pkill -15 -P $$ ; pkill -15 px4" SIGINT SIGTERM;

# Default positions
ENEMY_POS="3,3"
EGO_POS="0,0"

# Take custom position args if they exist
if [ "$2" != "" ]
then 
    ENEMY_POS=$1
    EGO_POS=$2
fi

# Get dir name
cur_dir=${PWD##*/}
echo $cur_dir
if [ $cur_dir == "missionapp" ]
then
    cd ../Firmware
    # Setup px4 for two drones
    ./Tools/sitl_multiple_run.sh 2;
    cd ../
    
    # Run simulators to connect to px4 at provided positions
    cd ./jMAVSim/
    JAVA_HOME=`/usr/libexec/java_home -v 1.8` java \
	     -Djava.ext.dirs= -jar ./out/production/jmavsim_run.jar \
	     -tcp :4560 -pos $ENEMY_POS -- -pos $EGO_POS \
	     -3d ./models/3dr_arducopter_quad_x_2.obj ;
    cd ../

    # Kill everything started in this script
    pkill -15 -P $$

    # These have a different PPID for some reason
    pkill -15 px4
    
    
else
    echo "Must be in missionapp directory to use this script."

    
fi



