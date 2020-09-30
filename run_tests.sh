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

MISSION=recon

# Catch SIGINT/SIGTERM and kill all spawned processes
trap "pkill -2 -P $$ ; pkill -2 px4; pkill -15 -P $$; exit;" SIGINT SIGTERM; 

# Max number of seconds per run (0 = no limit)
RUN_LIMIT=300

# Number of runs per case
RUNS_PER_CASE=1

# Coordinators
COORDINATORS=("PriorityCoordinator" "RobustnessCoordinator" "SynthRobustnessCoordinator")

if [ "$1" == "" ]
then
    echo "Invalid usage. Provide path to test directory as:"
    echo "run_tests.sh <test_dir> [num_cases]"
    exit
fi

# all cases
NUM_CASES=`ls ./$1/${COORDINATORS[0]} | wc -l`

if [ "$2" != "" ]
then
    echo "Only running $2 more cases... (and rerunning old stuff that needed to be rerun)"
    NUM_CASES=$2
fi

CASES_RUN=0

cur_dir=${PWD##*/}
echo $cur_dir

if [ $cur_dir == "missionapp" ]
then

    # Important that this is at appended to the end of the path  
    export MISSIONAPP=$PWD

    # Change to test dir
    cd ./$1;

    # Assume all coordinators have same config instances
    for CONFIG_INSTANCE in ./${COORDINATORS[0]}/*
    do
	
	# Trim to get the config directory
	CONFIG_INSTANCE=${CONFIG_INSTANCE##*/}
	
	if [ $CASES_RUN == $NUM_CASES ]
	then
	    echo "Ran $NUM_CASES. Finished."
	    exit
	fi

	# Only count this as a 'run' if the case hasn't been partially/totally done already
	if [ `ls ./*/$CONFIG_INSTANCE | grep --count "results"` != `ls . | wc -l` ]
	then
	    CASES_RUN=$CASES_RUN+1
	fi
	
	# For each coordinator, run this case
	for COORDINATOR in ${COORDINATORS[@]}
	do
	    pushd "./$COORDINATOR/$CONFIG_INSTANCE"
	    echo "Entering $PWD..."
	    
	    mkdir -p ./results;
	    for i in `seq $RUNS_PER_CASE`
	    do
		# Only run cases if they haven't been run before (allows for continuing script)
		if mkdir ./results/$i &> /dev/null ; then
		    # If setup fails, no data.json will be written. Keep trying again until we successfully collect data.
		    while [ ! -f ./results/$i/data.json ]
		    do

		    echo "Running case $CONFIG_INSTANCE for $COORDINATOR"
		    ENEMY_POS=`cat enemy_start_pos`
		    if [ RUNLIMIT == 0 ]
		    then

			MISSIONAPP=$MISSIONAPP $MISSIONAPP/run_multidrone.sh $ENEMY_POS 0,0 \
				  "--mission=$MISSION --coordinator=$COORDINATOR --outdir=./results/$i" \
			    &> ./results/$i/run.log
		    else
			# Run mission with fixed max time limit, kill after 2x time limit
			MISSIONAPP=$MISSIONAPP gtimeout -s SIGINT --kill-after=$RUN_LIMIT $RUN_LIMIT \
				  $MISSIONAPP/run_multidrone.sh $ENEMY_POS 0,0 \
				  "--mission=$MISSION --coordinator=$COORDINATOR --outdir=./results/$i" \
			    &> ./results/$i/run.log &
			
			# Need to run timeout in background and wait on its pid so that it can get SIGINT'd by us
			pid=$!

			# Need to run this in the background and wait on it so that it can be SIGINT'd by us
			wait $pid

			# Verify that everything is cleaned up okay
			if ps -u `id -un` -o comm | egrep -q "(missionapp|follower)"
			then			   
			   echo "Some dangling processes that need to be killed..."
			   pkill -9 follower java # DON'T KILL MISSIONAPP CUS IT MIGHT STILL BE LOGGING!
			fi

			# If enemy drone never wrote anything then we need to rm and restart
			if grep -q "Enemy drone velocity magnitude" ./results/$i/run.log
			then
			    echo "Test ran successfully. Results stored in $PWD/results/$i"
			else
			    echo "Run failed. Removing data.json and trying again in 30 seconds..."
			    rm ./results/$i/data.json
			    sleep 30 # Sleep to give everything a chance to recover
			fi
		    fi
		    done
		else
		    echo "Skipping case. /results already exists."
		fi
	    done
	    echo ""
	    popd	
	done
    done
fi
