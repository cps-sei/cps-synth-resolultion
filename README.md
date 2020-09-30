# Synthesis-based resolution of features/enforcers interactions in CPS

The code in this repository was used for [**Synthesis-Based Resolution of Feature Interactions in Cyber-Physical Systems**](https://conf.researchr.org/details/ase-2020/ase-2020-papers/75/Synthesis-Based-Resolution-of-Feature-Interactions-in-Cyber-Physical-Systems), published in ASE 2020.

## Prerequisites

Our experiments were developed and tested using MacOS. Everything should be able to run on Linux platforms as well, but this has not been tested and some installation instructions will be different. Refer to [**INSTALL.md**](INSTALL.md) for installation instructions.

## Running a single mission  

After following the instructions in [**INSTALL.md**](INSTALL.md), you should be able to run sample missions. 
The simplest way to run a single mission is using `run_multidrone.sh`. 

#### Troubleshooting: 

At the time of our project (mid 2019), we found that the drones didn't consistently take off successfully. This will result in notices from PX4 that the drone failed to arm drone or start offboard mode. We found that this can be mitigated system-by-system by choosing different sleep timings in `mission.cpp`, and by choosing different versions of `Firmware`. We found the most success using `9bfc4f2d540b3592b3c519d3bbf1e607e7374238` as our commit hash, but it is possible that newer versions are more stable and may not need the gross sleep timings used in our code.

## Running tests

For our data collection, we generated 500 randomly configured runs, which were each evaluated over the following resolution strategies: `SynthRobustness`, `Robustness`, and `Priority`. 

You can run these tests using the following commands: 

* `generate_tests.py <testdir_name> <num_configs>` Determine all combinations of variables, take a sample of `num_configs`, and make a folder for each case, finish.  
* `run_tests.py <testdir>` Go through each folder, run each case, log data. 
* `postprocess.py <testdir> <outfile>` Go through all collected data, put it all in a single json file
  * json file structured as follows:
    * `<coordinator> : <var> : [data1, data2, ... , dataN]`
    * where each `data1...dataN` is data collected from a single configuration. `N = num_configurations`–Therefore all vars are of the same length. Data indices across `var`'s make sense: i.e., they are referring to the same configuration/run.  

To run 500 configurations (1500 runs total across our 3 resolution strategies), we would run the following Python3 scripts (if we want our sample of configurations to be 500):

```bash
generate_tests.py experiment_data 500
run_tests.py experiment_data
postprocess.py experiment_data experiment_data.json
```

These commands will create `experiment_data.json`, which contains the run information to be processed in TODO.  It is likely that some runs will fail to start, and you may need to run `run_tests.py` another time. If many runs fail to start consider looking at the troubleshooting tips above.

Note that since each test can take up to 5 minutes to complete, running the above will take multiple days to complete. 
<details><summary>Click here for more detailed test information</summary> 
 
 ## More testing information  
 Tests can be stopped and restarted -- `run_tests.py` can functionally pick up where it left off. You can send a `SIGINT` to `run_tests.py` and it should quit, log data that it has collected so far, and clean up all of the spawned processes. BEWARE: if you want to stop a test halfway through, it will log what was collected so far, and when you continue the tests later this case will be considered complete (bc there are log files there). You should manually clean these log files (remove the `results` directory) if you wish for the tests for that case to be completed. 

There is a hard limit for how long each case runs. Currently this is set at 5 minutes. You can change this at the top of `run_tests.py`.

Output will be stored in `<coord_name>/CASE_NAME/results/1/`

Most data that is going to be used for evaluation is stored in JSON format. Names of keys aren't great, but most are pretty self-explanatory. Some less obvious explanations are as follows:

* `max_violation_length` refers to the maximum number of ticks in a row the drone was in violation of a given property. 
* `Boundary` property includes information about both the ticks that the boundary property was violated (time to interception too low) AND ticks that the actual boundary was physically violated by the ego drone. 
 </details>
<details>
<summary>Click here for more detailed run information</summary> 

## More run information  

For details about what this script is actually doing, you can refer to the code in `(run|setup)_multidrone.sh` – The important bits are readable.

For details about the multidrone infrastructure generally, refer to `multi-drone-px4-apps.pdf` – It is very useful.

You can also pass the following arguments to `run_multidrone.sh` :

* <enemyX, enemyY egoX, egoY>
* <"missionapp args">
  - `--coordinator=(SynthRobustness|Robustness|Conjunction|Priority|Weighted|Intersecting|Simple)Coordinator`
  - `--mission=(recon|flyeight)`
  - `--outdir=<relativepath>` Where to dump output files
  - **Note: missionapp args need to be in quotes so they are parsed as a single script arg**

Example: 

`run_multidrone 0,0 3,3 "--coordinator=SynthRobustnessCoordinator --mission=recon --outdir=./out"`

The above arguments will run a mission where the enemy starts at (x=0,y=0), the ego drone starts at (x=3,y=3), conflicts are resolved using synthesis, they perform a recon mission, and the logs are written to the `./out` directory. 

Many other run configurations be controlled using the `drone.cfg` file. 

</details>

<details>
 <summary>More about logging</summary> 
 
## Logging

Currently log the following:  

For plot.py:

* `plot_enemy_drone.dat`
* `plot_my_drone.dat`
* `plot_coordinated_points.dat`

For plot_animation.py:

* `plot_animation_data_ego.dat` for plot_animation.py
* `plot_animation_data_enemy.dat` for plot_animation.py

For general data collection:

* `statestore.log` write everything from statestore. Each line corresponds to the state of a given tick and contains the following: 

  ```
  ego_x_pos ego_y_pos ego_z_pos 
  ego_x_vel ego_y_vel ego_z_vel 
  enemy_x_pos enemy_y_pos enemy_z_pos 
  enemy_x_vel enemy_y_vel enemy_z_vel
  ```

* `coordinator_activity.dat` robustness value of each enforcer at each tick
</details>

<details>
 <summary>More about how to use run data</summary> 
  
## Using data

Visualize ego, enemy, and coordinated points (ignore z pos):  
`python3 plot.py plot_my_drone.dat plot_enemy_drone.dat plot_coordinated_points.dat`
Visualize ego, enemy in real time (include z pos):
`python3 plot_animation.py plot_animation_data_ego.dat plot_animation_data_enemy.dat`

  * Note: can use spacebar to pause/play animation

Code hasn't been written to extract meaningful information from general log files. Relevant information may include:

* Minimum robustness value for each enforcer
* Maximum boundary violation, number of times being caught, max time in missile zone at inappropriate height
* Avg global robustness value? Note: for this would need to know weights when logging OR would need to determine weights from the config file which would be slightly annoying but not that bad
  * May be easier to just do avg robustness value for each property?
  
  <details><summary>More about `drone.cfg` usage. </summary> 

## drone.cfg usage
  - Config file should be called 'drone.cfg', and should be accessible via relative path wherever you are executing. Follower will also need to parse this, so follower and missionapp should be called from the same directory or you need to setup your path variable to make this work. (or you can just change how this works)  
    
- An example config file is provided at 'drone.cfg'
  
- The config file / parsing situation is really terribly setup – Currently only numerical values can be used (i.e., no bool/strings), and whitespace will break things.
  
- Variables are mostly self-explanatory. Units are meters for space variables, seconds for time variables, m/s for rate variables. 
  
- To add a new variable, you need to do the following (gross and annoying, sorry):
  - Add the variable as `extern <type> <NEW_VAR_NAME>;` in `DroneUtil.h`
  - Provide the default value as `<type> <NEW_VAR_NAME> = <default_value>; ` in `DroneUtil.cpp`
  - Add the variable to the long `setVar` function in `DroneUtil.cpp`
  </details>

# Implementation details
<details><summary>About missions</summary> 

## Missions

arm, takeoff, start offboard via `mission::setup()`  
perform mission using `mission::run()`  
write to log files using `mission::log()`

When writing missions, in most cases you should only need to redefine `run()`. When making new missions, keep in mind:

* You will have a main loop that sends a new action every tick. It's not a great setup right now, so you need to manually ensure some things. 
  * sleep in your loop for `TICK_DURATION` seconds. Coordination sometimes takes time, so you should only sleep for the difference between `TICK_DURATION` and computation time (e.g., only sleep for 2 ms if your tick duration is 60 ms and your coordinator took 58 ms)
  * You are responsible for writing to StateStore every tick. 

#### flyeightmission

Fly in a figure eight. If you wish to change the size of the figure 8, you must do so within the file.  

Mission duration: However long it would theoretically take to fly the figure 8 at max speed. This is determined at mission start, and the mission will be terminated after this time limit. 

#### reconmission

* Fly to a set of waypoints and then fly to (0,0). 
* Upon reaching a waypoint, try to perform a recon mission (get down to some altitude and fly in a figure 8), and then go back up to normal altitude. This can be interrupted. As long as it reached the waypoint initially, it will consider the waypoint complete.
* Waypoints randomly generated within boundary according to `WAYPOINT_SEED` in config file. 
* At `WAYPOINT_SEED=0`, the waypoints form a box within the boundary, and the drone flies to each point in a counterclockwise order, starting with the first quadrant (i.e., the top right corner – x,y > 0). 
* Mission duration: Potentially nonterminating. Finishes when drone returns to (0,0) after recon-ing all waypoints. 
* Assumes boundary is centered around origin
</details>

<details>
 <summary>About Signal Functions</summary>
 
## SigFuns

* Need to specify maximum and minimum robustness value. These should be the maximum and minimum with respect to values that you care about. e.g., the maximum for the Boundary TTI Fun should NOT be infinity. It should be the maximum value _that we should be sensitive to_. So beyond N times the safe threshold, we do not care about
* Robustness values are scaled within [-1,1]
* Negative robustness values are then fit to the following curve:  
  `-((a^-x)-1)/(a-1)+x`
  * Maps from `[-1,0] ⟶ [-2,0]`, exponential decrease in robustness while maintaining the following: `forall x. (x in range [-1,0]) ⇒ (f(x) <= x)`
</details>
<details><summary>More about Enforcers</summary>  

## Enforcers

All can suggest a range of actions through the `SUGGEST_ACTION_RANGE`config variable. This was toggled off for experiments. 

#### BoundaryEnforcer

* SigFun: Time To Interception (TTI)
* Goal: Stay within 3d box
* Under violation: Try to fly towards origin
* Assumes: 3d box centered around origin  


#### RunawayEnforcer

* SigFun: Distance To Target (DTT)
* Goal: Stay away from enemy drone
* Under violation: Try to fly away from enemy
* Assumes: Flat ground

#### `(Missile|Recon)`Enforcer

* SigFun: ReconFun (shared by ReconEnforcer, bad name)
* Goal: Within a certain xy boundary, try to stay at a certain altitude. 
* Under violation: Go towards the desired altitude
* Assumes: Flat ground

#### FlightEnforcer

* SigFun: Distance To Ground (DTG)
* Goal: Don't crash into the ground
* Under violation: Go up
* Assumes: Flat ground

</details>

<details>
 <summary>About coordinators (i.e., resolution strategies)</summary>  

## Coordinators

#### RobustnessCoordinator

Pick the most globally satisfying action of the conflicted actions, or among conflicted actions and the synthesized actions if `SYNTHESIZE_ACTIONS=1`.

##### High-level overview:

* Determine action range: `([min_x, min_y, min_z], [max_x, max_y, max_z])`
* Uniformly randomly select vectors within this range. (note: right now, sampling each component from its own generator)
  * Number of candidate vectors to select is dependent on the size of the range of actions. Right now, within a dimension, the range is multiplied by some number S. (e.g., S=10, then we will sample 10 vectors per 1 unit of range). So with the range `([-1, -1, -1], [1,1,1]`, we would randomly sample `20*20*20` candidate vectors. This will probably take more than `TICK_DURATION` 
* Estimate signal given an action `estimate_signal()`
  * Signal estimate makes best-attempt at estimating state that would be useful for comparing robustness values
  * To estimate signal, we need to be able to estimate the actual next velocity given our current velocity and the new velocity (drones can't change velocities instantaneously, sadly)
    * Some estimated acceleration is determined, and we calculate where we would be a few ticks into the future (so that the drone will have a chance to actually change its position, providing more meaningful results to SigFuns that rely only on positions (not just velocities))
    * We update the ego drone position \& velocity by `TICKS_TO_CORRECT` ticks, but don't update the enemy drone position \& velocity that much. This is because functionally it will provide the same relative robustness values and is far simpler. 
* We can toggle `CHOOSE_LEAST_DIFFERENT_ACTION` in conjunction with `SUGGEST_ACTION_RANGES` to get smoother runs by choosing the action from the set of actions (if no conflict) that is most similar to the original mission action

#### PriorityCoordinator

Given a set of conflicted features, the action produced by the feature with the highest weight will be selected. 
</details>
 
<details>
 <summary>Implementation details to know if you want to extend this code</summary> 

## Unpleasant implementation details

* `x` refers to `north`, `y` refers to `east`, `z` refers to `down`. This is know as [NED coordinates](https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates#Local_north,_east,_down_(NED)_coordinates).
  * NOTE: `down` is a confusing position component, because actual (relative) altitude is `-down`  
    (e.g., 2 m above the ground corresponds to ` down=-2`) 
* The current config file situation is terrible
* Leading/trailing whitespace in the strings of `Signal::value()` lookups can cause distress – It may silently provide you the value of something else.
* When adding variables to the config file, you need to add it in like 3 places. Beware copy/paste in setVar because you want to set the right variable and this can fail silently.
</details>
