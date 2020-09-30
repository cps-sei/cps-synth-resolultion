#!/usr/bin/env python3
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

import sys
import os
import numpy as np
import pathlib
import itertools
import random
import json

# Assume ego starts 0,0 always
# Just relative to ego at varying distances
enemy_start_positions = [
    '5,5',
    '5,0',
    '0,5',
    '-5,5',
    '5,-5',
    '-5,-5',
    '-5,0',
    '0,-5',

    '1,1',
    '1,0',
    '0,1',
    '-1,1',
    '1,-1',
    '-1,-1',
    '-1,0',
    '0,-1',

    '10,10',
    '10,0',
    '0,10',
    '-10,10',
    '10,-10',
    '-10,-10',
    '-10,0',
    '0,-10',
    ]
    
# Config values that will be changed
config_vals = {
    'ENEMY_DRONE_SPEED'    : [x for x in np.arange(1.2, 2.1, 0.1)],
    'WAYPOINT_SEED'        : [x for x in range(0, 999)], # Kinda dumb way to do this space-wise but it's fine
    'BOUNDARY_SIZE'        : [x for x in np.arange(10, 30, 1)],
    #        'SUGGEST_ACTION_RANGE' : [0,1]
}

# FLIGHT_WEIGHT stays constant
weight_vals = [
    # Equal weights
    {'BOUNDARY_WEIGHT' : 1,'RUNAWAY_WEIGHT'  : 1,  'MISSILE_WEIGHT'  : 1},

    # 1 : 1.5 : 2
    {'BOUNDARY_WEIGHT' : 1,'RUNAWAY_WEIGHT'  : 1.5,'MISSILE_WEIGHT'  : 2},
    {'BOUNDARY_WEIGHT' : 1,'MISSILE_WEIGHT'  : 1.5,'RUNAWAY_WEIGHT'  : 2},
    
    {'RUNAWAY_WEIGHT' : 1,'BOUNDARY_WEIGHT'  : 1.5,'MISSILE_WEIGHT'  : 2},
    {'RUNAWAY_WEIGHT' : 1,'MISSILE_WEIGHT'   : 1.5,'BOUNDARY_WEIGHT' : 2},

    {'MISSILE_WEIGHT' : 1,'RUNAWAY_WEIGHT'   : 1.5,'BOUNDARY_WEIGHT' : 2},
    {'MISSILE_WEIGHT' : 1,'BOUNDARY_WEIGHT'  : 1.5,'RUNAWAY_WEIGHT'  : 2},

    # 1 : 2 : 3
    {'BOUNDARY_WEIGHT' : 1,'RUNAWAY_WEIGHT'  : 2,'MISSILE_WEIGHT'  : 3},
    {'BOUNDARY_WEIGHT' : 1,'MISSILE_WEIGHT'  : 2,'RUNAWAY_WEIGHT'  : 3},
    
    {'RUNAWAY_WEIGHT' : 1,'BOUNDARY_WEIGHT'  : 2,'MISSILE_WEIGHT'  : 3},
    {'RUNAWAY_WEIGHT' : 1,'MISSILE_WEIGHT'   : 2,'BOUNDARY_WEIGHT' : 3},

    {'MISSILE_WEIGHT' : 1,'RUNAWAY_WEIGHT'   : 2,'BOUNDARY_WEIGHT' : 3},
    {'MISSILE_WEIGHT' : 1,'BOUNDARY_WEIGHT'  : 2,'RUNAWAY_WEIGHT'  : 3},
]
        
def make_config_file(base_config_file, outfile, vals):
    # Open input and output file
    with open(base_config_file, 'r') as base, open(outfile, 'w') as out:
        # Convert to list by space delim
        for line in base:
            line_lst = line.split(' ')
            
            # If this var is one we change, then write what's stored in vars
            if(line_lst[0] in vals):
                # Handle the case that it's a float differently bc annoying precision
                if isinstance(line_lst[0], np.float64):
                    out.write(line_lst[0] + ' ' + '{:.2f}'.format(vals[line_lst[0]]) + '\n')
                else:
                    out.write(line_lst[0] + ' ' + str(vals[line_lst[0]]) + '\n')
            # If this var is not one we change, write it as is
            else:
                out.write(line)

def default(o):
    if isinstance(o, np.int64): return int(o)  
    raise TypeError

def make_files(config, rootdir, enemy_start_positions, num_configurations):
    newdir = ''

    # They're all the same at this point -- just get the vals for any coordinator
    vals = config["RobustnessCoordinator"]
    
    # Get all the combinations of variable-values we have
    combinations = [dict((zip(vals.keys(), t))) for t in itertools.product(*vals.values())]
        
    sample_size = num_configurations
        
    # Get a random sample of the combinations
    comb_sample = random.sample(combinations, sample_size)

    # Randomly assign weights to each case
    for entry in comb_sample:
        weights = random.choice(weight_vals)
        entry.update(weights)

    for coordinator in config:
        try:
            newdir = rootdir+'/'+coordinator
            os.makedirs(newdir, exist_ok=True)
        except OSError:
            print("Failed to create directory: %s" % newdir)

        i=0

        # Create a directory and a corresponding config file for each test case
        for entry in comb_sample:

            # Everything else is random so this is fine
            enemy_start_pos_str = enemy_start_positions[i%len(enemy_start_positions)]
            i = i+1
            
            # Need to add this manually bc it's not in config file params
            controlled_vars = entry.copy();
            controlled_vars["enemy_strt_pos"] = enemy_start_pos_str;

            dirname = ''
            for name in entry:
                if isinstance(entry[name], np.float64):
                    dirname+=name+'{:.2f}'.format(entry[name])+'-'
                else:
                    dirname+=name+str(entry[name])+'-'
                    
            # trim the hyphen off the end
            dirname = dirname[0:-1]
            
            try:
                os.makedirs(rootdir+'/'+coordinator+'/'+dirname, exist_ok=True)
            except OSError:
                print("Failed to create directory: %s" % rootdir+'/'+coordinator+'/'+dirname)

            # Write config file
            make_config_file('./drone.cfg', rootdir+coordinator+'/'+dirname+'/'+'drone.cfg', entry)

            with open(rootdir+coordinator+'/'+dirname+'/'+'controlled_vars.json', 'w') as controlled_varsfile:
                json.dump(controlled_vars, controlled_varsfile, default=default)
            # Write positions
            with open(rootdir+coordinator+'/'+dirname+'/'+'enemy_start_pos', 'wb') as posfile:
                posfile.write(enemy_start_pos_str.encode('utf-8'))

def main():
    if(len(sys.argv) != 3):
        print("Usage: generate_tests.py <test_dir> <num_configurations>")
        exit()
    cwd = os.getcwd()
    os.makedirs(sys.argv[1], exist_ok=True)
    num_configurations = int(sys.argv[2])
    rootdir = cwd+'/'+sys.argv[1]+'/'
    
    if((cwd.split('/'))[-1] != 'missionapp'):
        print("Must be in missionapp directory to use this script. Given %s\n" % cwd)
        print(cwd.split('/'))
        exit()

    base_config_file = 'drone.cfg'

    coordinators = ['PriorityCoordinator', 'RobustnessCoordinator', 'SynthRobustnessCoordinator']
    
    config = { coord : config_vals.copy() for coord in coordinators }

    make_files(config, rootdir, enemy_start_positions, num_configurations)
    
if __name__ == "__main__":
    main()
