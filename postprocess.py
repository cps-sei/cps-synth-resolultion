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

TICK_DURATION = 0.06 * 1000

# NOTE: this assumes fixed positions in the statestore
def check_enemy_drone_alive(dirname):
    positions  = []
    velocities = []
    last_x = last_y = last_z = None
    total_diff = 0
    
    with open(dirname+'/statestore.log') as statestore:
        for line in statestore:
            components = line.split(',')
            if(len(components) <= 2):
                continue
            enemy_x = float(components[6])
            enemy_y = float(components[7])
            enemy_z = float(components[8])
            positions.append((enemy_x, enemy_y, enemy_z))

            if last_x != None:
                total_diff += (abs(enemy_x - last_x) + abs(enemy_y - last_y) + abs(enemy_z - last_z))

            last_x = enemy_x
            last_y = enemy_y
            last_z = enemy_z

            enemy_x_vel = float(components[9])
            enemy_y_vel = float(components[10])
            enemy_z_vel = float(components[11])
            velocities.append((enemy_x_vel, enemy_y_vel, enemy_z_vel))

    if(len(list(filter(lambda x: abs(x[0]) + abs(x[1]) + abs(x[2]) >= 0.1, velocities))) < 5):
        print('\033[93mWARNING: ' + dirname + 'probably has bunk data. Velocities too low. Check it, and probably remove results and rerun tests.\x1b[0m\n')
    elif(total_diff < 5):
        print('\033[93mWARNING: ' + dirname + 'probably has bunk data. Drone didn\'t move enough. Check it, and probably remove results and rerun tests.\x1b[0m\n')

def get_overhead_info(dirname):
    overhead_info = {
        'time_taken_per_tick' : [],
        'absolute_overhead_per_tick' : [],
        'relative_overhead_per_tick' : []
    }
    with open(dirname+'/run.log') as output:
        for line in output:
            if("Coordinator took" in line):
                time_taken = int(line.split(' ')[2]) # milliseconds
                overhead_info['time_taken_per_tick'].append(time_taken)
                if time_taken > TICK_DURATION:
                    overhead_info['absolute_overhead_per_tick'].append(time_taken - TICK_DURATION)
                    overhead_info['relative_overhead_per_tick'].append((time_taken-TICK_DURATION) / TICK_DURATION)
                else:
                    overhead_info['absolute_overhead_per_tick'].append(0)
                    overhead_info['relative_overhead_per_tick'].append(0)
    return overhead_info
    
def main():
    if(len(sys.argv) != 3):
        print("Usage: postprocess.py <test_dir> <outfile>")
        exit()
    cwd = os.getcwd()
    test_dir = sys.argv[1]
    outfile = sys.argv[2]

    broken = False
    
    # Merge everything together
    all_data = {}

    coordinators = os.listdir(test_dir)
    
    for coordinator_dir in os.listdir(test_dir):
        print(coordinator_dir)
        all_data[coordinator_dir] = {}
        
        for case_dir in os.listdir(test_dir + '/' + coordinator_dir):
            # If there are no results for any coordinator wrt this case, then assume this is the end of data and we're done
            if(all(not os.path.exists(test_dir+'/'+coordinator+'/'+case_dir+'/results') for coordinator in coordinators)):
                print("No coordinators have data for " + case_dir + ". Assuming this is end of data, stopping data aggregation here for " + coordinator_dir + ".")
                break
            
            try:
                check_enemy_drone_alive(test_dir+'/'+coordinator_dir+'/'+case_dir+'/results/1/')
                overhead_info = get_overhead_info(test_dir+'/'+coordinator_dir+'/'+case_dir+'/results/1/')
                all_data[coordinator_dir].update(overhead_info)
                with open(test_dir+'/'+coordinator_dir+'/'+case_dir+'/controlled_vars.json', 'r') as data:
                  jsondata = json.load(data)
                  if 'controlled_vars' not in all_data[coordinator_dir]:
                    all_data[coordinator_dir]['controlled_vars'] = [jsondata]
                  else:
                    all_data[coordinator_dir]['controlled_vars'].append(jsondata)
                with open(test_dir+'/'+coordinator_dir+'/'+case_dir+'/results/1/coordinator_activity.json', 'r') as data:
                    jsondata = json.load(data)
                    for prop in jsondata:
                        if prop not in all_data[coordinator_dir]:
                            all_data[coordinator_dir][prop] = {}
                        for key in jsondata[prop]:
                            if key not in all_data[coordinator_dir][prop]:
                                all_data[coordinator_dir][prop][key] = jsondata[prop][key]
                            else:
                                all_data[coordinator_dir][prop][key].extend(jsondata[prop][key])
                with open(test_dir+'/'+coordinator_dir+'/'+case_dir+'/results/1/data.json', 'r') as data:
                    jsondata = json.load(data)
                    for prop in jsondata:
                        if prop not in all_data[coordinator_dir]:
                            all_data[coordinator_dir][prop] = {}
                        for key in jsondata[prop]:                        
                            if key not in all_data[coordinator_dir][prop]:
                                all_data[coordinator_dir][prop][key] = jsondata[prop][key]
                            else:
                                all_data[coordinator_dir][prop][key].extend(jsondata[prop][key])
                with open(test_dir+'/'+coordinator_dir+'/'+case_dir+'/results/1/run_data.json', 'r') as data:
                    jsondata = json.load(data)
                    for key in jsondata:
                        if key not in all_data[coordinator_dir]:
                            all_data[coordinator_dir][key] = jsondata[key]
                        else:
                            all_data[coordinator_dir][key].extend(jsondata[key])
                                
            except FileNotFoundError as e:
                # This needs to be fatal, because otherwise certain cases could be missing and indices across properties/coordinators would get messed up
                print('\033[93mNOTICE: ' + e.filename + ' not found. Remove /results from this directory and rerun run_tests.sh.' + '\x1b[0m')
                broken = True

    if not broken:
        num_cases = len(all_data[coordinators[0]]['controlled_vars'])
        if not all(len(all_data[coordinator]['controlled_vars']) == num_cases for coordinator in coordinators):
            print('Note: not all coordinators have the same number of cases. Can resolve this by trimming your data or just running 1 more case using the `run_tests.sh` script.')
            print(list([coordinator + " : " + str(len(all_data[coordinator]['controlled_vars'])) for coordinator in coordinators]))
        else:
            print('Collected data for ' + str(num_cases) + ' cases for each coordinator.')
        # Write everything out
        with open(outfile, 'w') as outfile:
            json.dump(all_data, outfile)
    else:
        print("Failed to write file. Fix broken /results directories and rerun this script.")
    
if __name__ == "__main__":
    main()
