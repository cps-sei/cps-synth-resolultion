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
trap "pkill -15 -P $$ ; pkill -15 px4; pkill -9 follower" SIGINT SIGTERM;

# Default positions
ENEMY_POS="3,3"
EGO_POS="0,0"

# Missionapp args
ARGS="--mission=flyeight --coordinator=RobustnessCoordinator"

if [ "$MISSIONAPP" == "" ]
then
    MISSIONAPP=$PWD
fi

# Take custom position args if they exist
if [ "$2" != "" ]
then 
    ENEMY_POS=$1
    EGO_POS=$2
fi

# Take a third mission argument if it exists
if [ "$3" != "" ]
then
    ARGS=$3
fi

# Get dir name
cur_dir=${MISSIONAPP##*/}
echo $cur_dir
if [ $cur_dir == "missionapp" ]
then
    pushd $MISSIONAPP/../Firmware
    # Setup px4 for two drones
    ./Tools/sitl_multiple_run.sh 2;
    popd;
    
    # Run simulators to connect to px4 at provided positions
    pushd $MISSIONAPP/../jMAVSim/
    JAVA_HOME=`/usr/libexec/java_home -v 1.8` java \
	     -Djava.ext.dirs= -jar out/production/jmavsim_run.jar \
	     -tcp :4560 -pos $ENEMY_POS -- -pos $EGO_POS \
	     -3d models/3dr_arducopter_quad_x_2.obj &
    popd;

    # Give some time for the simulator to start up

    sleep 20 && pushd $MISSIONAPP/../mavlink-router;
    # Route px4->ego_drone to both the ego drone and the follower drone 
    ./mavlink-routerd -e 127.0.0.1:14590 -e 127.0.0.1:14591 -t 0 0.0.0.0:14541 &

    # Route px4->follower to both the ego drone and the follower drone 
    ./mavlink-routerd -e 127.0.0.1:14691 -e 127.0.0.1:14690 -t 0 0.0.0.0:14540 &
    
    popd;

	
    # Run
    sleep 5 && $MISSIONAPP/follower/follower udp://:14690 udp://:14590 &
    sleep 1 && $MISSIONAPP/missionapp --multiprocess $ARGS udp://:14591 udp://:14691;
    
    # Kill everything started in this script
    pkill -15 -P $$

    # These have a different PPID for some reason
    pkill -15 px4
else
    echo "Must be in missionapp directory to use this script."  
fi



