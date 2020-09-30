/*
 * Synthesis-based resolution of features/enforcers interactions in CPS
 * Copyright 2020 Carnegie Mellon University.
 * NO WARRANTY. THIS CARNEGIE MELLON UNIVERSITY AND SOFTWARE ENGINEERING
 * INSTITUTE MATERIAL IS FURNISHED ON AN "AS-IS" BASIS. CARNEGIE MELLON
 * UNIVERSITY MAKES NO WARRANTIES OF ANY KIND, EITHER EXPRESSED OR IMPLIED,
 * AS TO ANY MATTER INCLUDING, BUT NOT LIMITED TO, WARRANTY OF FITNESS FOR
 * PURPOSE OR MERCHANTABILITY, EXCLUSIVITY, OR RESULTS OBTAINED FROM USE OF
 * THE MATERIAL. CARNEGIE MELLON UNIVERSITY DOES NOT MAKE ANY WARRANTY OF ANY
 * KIND WITH RESPECT TO FREEDOM FROM PATENT, TRADEMARK, OR COPYRIGHT
 * INFRINGEMENT.
 * Released under a BSD (SEI)-style license, please see license.txt or contact
 * permission@sei.cmu.edu for full terms.
 * [DISTRIBUTION STATEMENT A] This material has been approved for public
 * release and unlimited distribution.  Please see Copyright notice for
 * non-US Government use and distribution.
 * This Software includes and/or makes use of the following Third-Party Software
 * subject to its own license:
 * 1. JsonCpp
 * (https://github.com/open-source-parsers/jsoncpp/blob/master/LICENSE)
 * Copyright 2010 Baptiste Lepilleur and The JsonCpp Authors.
 * DM20-0762
 */

#include <assert.h>
#include <vector>
#include <dronecode_sdk/offboard.h>
#include <math.h>
#include <random>
#include <time.h>

#include "RobustnessCoordinator.h"
#include "StlEnforcer.h"
#include "DroneUtil.h"
#include <iostream>
#include "Signal.h"
#include "StlExpr.h"

using namespace dronecode_sdk;
using namespace cdra;
using namespace std;

struct Pos3d {
  float x;
  float y;
  float z;
};

/* Return the least different vector (using cosine similarity as measure of similarity) */
Offboard::VelocityNEDYaw get_least_different(const Offboard::VelocityNEDYaw original,
					     const vector<Offboard::VelocityNEDYaw>& candidates) {
  double max = droneutil::cosineSimilarity(original, candidates[0]);
  auto argmax = candidates[0];

  double cur_value;
  for(auto cur_velocity : candidates) {
    cur_value = droneutil::cosineSimilarity(cur_velocity, original);
    if(cur_value > max) {
      max = cur_value;
      argmax = cur_velocity;
    }
  }

  return argmax;
}

dronecode_sdk::Offboard::VelocityNEDYaw update_velocity(dronecode_sdk::Offboard::VelocityNEDYaw& old_v, dronecode_sdk::Offboard::VelocityNEDYaw& new_v, float num_steps) {
  dronecode_sdk::Offboard::VelocityNEDYaw ret_v = old_v;
  float td = droneutil::TICK_DURATION;
  float est_acc = 2;

  /* Determine if inc/decrease in velocity */
  int x_dir = (new_v.north_m_s < old_v.north_m_s ? -1 : 1);
  int y_dir = (new_v.east_m_s  < old_v.east_m_s  ? -1 : 1);
  int z_dir = (new_v.down_m_s  < old_v.down_m_s  ? -1 : 1);

  /* Figure out the max possible change given some acceleration */
  ret_v.north_m_s += x_dir * est_acc * td * num_steps;
  ret_v.east_m_s  += y_dir * est_acc * td * num_steps;
  ret_v.down_m_s  += z_dir * est_acc * td * num_steps;

  /* Velocity will be updated to the least extreme value 
   * between max possible change and desired change */
  ret_v.north_m_s = x_dir > 0 ?
    min(ret_v.north_m_s, new_v.north_m_s) :
    max(ret_v.north_m_s, new_v.north_m_s);
  ret_v.east_m_s = y_dir > 0 ?
    min(ret_v.east_m_s, new_v.east_m_s) :
    max(ret_v.east_m_s, new_v.east_m_s);
  ret_v.down_m_s = z_dir > 0 ?
    min(ret_v.down_m_s, new_v.down_m_s) :
    max(ret_v.down_m_s, new_v.down_m_s);
  
  return ret_v;
}

/*
 * Could get more accurate estimates if we update both drones together 
std::Pair<dronecode_sdk::Telemetry::PositionVelocityNED, dronecode_sdk::Telemetry::PositionVelocityNED> update_both_drones(args) {

}
*/

void get_est_signal(Signal* cur_signal,
		    dronecode_sdk::Offboard::VelocityNEDYaw action,
		    Signal* est_signal) {
  // NOTE: Giving inaccurate position/velocity estimates for next state -- accuracy only really matters with respect to robustness relative to other potential actions. i.e., as long as this estimate roughly maintains the ordering of r(a_1') ... r(a_n') we're okay.

  float td = droneutil::TICK_DURATION;

  const float vel_east_m_s  = est_signal->value("vel_east_m_s");
  const float vel_north_m_s = est_signal->value("vel_north_m_s");
  const float vel_down_m_s  = est_signal->value("vel_down_m_s");

  dronecode_sdk::Offboard::VelocityNEDYaw old_v;
  
  old_v.north_m_s = vel_north_m_s;
  old_v.east_m_s  = vel_east_m_s;
  old_v.down_m_s  = vel_down_m_s;

  auto new_action = update_velocity(old_v, action, droneutil::TICKS_TO_CORRECT);

  dronecode_sdk::Offboard::VelocityNEDYaw enemy_vel{
      cur_signal->value("enemy_vel_east_m_s"),
      cur_signal->value("enemy_vel_north_m_s"),
      cur_signal->value("enemy_vel_down_m_s"),
      0
  };

  /* Note: This estimate assumes that the new velocity is used immediately, 
   * which is likely not the case, but should be an okay simple estimate. */
  const float new_pos_east  = cur_signal->value("pos_east_m")  +
    (((new_action.east_m_s))  * td * droneutil::TICKS_TO_CORRECT);
  const float new_pos_north = cur_signal->value("pos_north_m") +
    (((new_action.north_m_s)) * td * droneutil::TICKS_TO_CORRECT);
  const float new_pos_down  = cur_signal->value("pos_down_m")  +
    (((new_action.down_m_s))  * td * droneutil::TICKS_TO_CORRECT);

  
  int ticks_in_old_dir = 2;
  
  /* Enemy goes N ticks in the old direction */
  float new_enemy_pos_east  = cur_signal->value("enemy_pos_east_m")  +
    enemy_vel.east_m_s*td*ticks_in_old_dir;
  float new_enemy_pos_north = cur_signal->value("enemy_pos_north_m") +
    enemy_vel.north_m_s*td*ticks_in_old_dir;
  float new_enemy_pos_down  = cur_signal->value("enemy_pos_down_m")  +
    enemy_vel.down_m_s*td*ticks_in_old_dir;

  /* NOTE: This makes 'side' moves less effective */
  const float delta_east  = new_pos_east  - new_enemy_pos_east;
  const float delta_north = new_pos_north - new_enemy_pos_north;
  const float delta_down  = new_pos_down  - new_enemy_pos_down;
  
  const float delta = sqrt(pow(delta_east , 2.0) +
			   pow(delta_north, 2.0) +
			   pow(delta_down , 2.0));

  const int enemy_speed = droneutil::ENEMY_DRONE_SPEED;

  Offboard::VelocityNEDYaw attempted_enemy_vel{
      enemy_speed*(delta_north/delta),
      enemy_speed*(delta_east/delta),
      enemy_speed*(delta_down/delta)
      };
  
  auto new_enemy_action = update_velocity(enemy_vel, attempted_enemy_vel, droneutil::TICKS_TO_CORRECT-ticks_in_old_dir);
  /* This will update the enemy position further -- Currently leaving the enemy position largely the same...
  new_enemy_pos_east  +=
    (((new_enemy_action.east_m_s))  * td * (droneutil::TICKS_TO_CORRECT - ticks_in_old_dir));
  new_enemy_pos_north +=
    (((new_enemy_action.north_m_s)) * td * (droneutil::TICKS_TO_CORRECT - ticks_in_old_dir));
  new_enemy_pos_down  +=
    (((new_enemy_action.down_m_s))  * td * (droneutil::TICKS_TO_CORRECT - ticks_in_old_dir));
  */
  est_signal->append({
      new_pos_east, new_pos_north, new_pos_down,
      new_action.east_m_s, new_action.north_m_s, new_action.down_m_s,
      new_enemy_pos_east , new_enemy_pos_north , new_enemy_pos_down,
      new_enemy_action.east_m_s, new_enemy_action.north_m_s,
      new_enemy_action.down_m_s
        }
    );
}

Offboard::VelocityNEDYaw get_action_in_range(pair<Offboard::VelocityNEDYaw, Offboard::VelocityNEDYaw>& vels) {
  
  static std::random_device rd;
  static std::mt19937 gen(rd());
  
  std::uniform_real_distribution<float> dis_north(vels.first.north_m_s,
						  vels.second.north_m_s);
  std::uniform_real_distribution<float> dis_east (vels.first.east_m_s ,
						  vels.second.east_m_s );
  std::uniform_real_distribution<float> dis_down (vels.first.down_m_s ,
						  vels.second.down_m_s );
  /* Just choose within entire range
  std::uniform_real_distribution<float> dis_north(-1, 1);
  std::uniform_real_distribution<float> dis_east (-1, 1);
  std::uniform_real_distribution<float> dis_down (-1, 1);
  */
  
  Offboard::VelocityNEDYaw ret {
    dis_north(gen),
    dis_east (gen),
    dis_down (gen),
  };

  return ret;
}

pair<Offboard::VelocityNEDYaw, Offboard::VelocityNEDYaw> get_action_range(vector<Offboard::VelocityNEDYaw> actions) {
  Offboard::VelocityNEDYaw min, max;
  min = actions[0];
  max = actions[0];
  
  for(auto action : actions) {
    if(action.north_m_s < min.north_m_s) { min.north_m_s = action.north_m_s; }
    if(action.east_m_s  < min.east_m_s ) { min.east_m_s  = action.east_m_s;  }
    if(action.down_m_s  < min.down_m_s ) { min.down_m_s  = droneutil::EGO_Z_VELOCITY ? action.down_m_s : 0;   }
    if(action.north_m_s > max.north_m_s) { max.north_m_s = action.north_m_s; }
    if(action.east_m_s  > max.east_m_s ) { max.east_m_s  = action.east_m_s;  }
    if(action.down_m_s  > max.down_m_s ) { max.down_m_s  = droneutil::EGO_Z_VELOCITY ? action.down_m_s : 0;   }
  }
  
  return { min, max };
}

Offboard::VelocityNEDYaw weightedMergeCommands(Offboard::VelocityNEDYaw& fst,
					       const dronecode_sdk::Offboard::VelocityNEDYaw& other, float weight) {
  Offboard::VelocityNEDYaw retNED;
  retNED.north_m_s = weight * fst.north_m_s + (1 - weight) * other.north_m_s;
  retNED.east_m_s  = weight * fst.east_m_s  + (1 - weight) * other.east_m_s;
  retNED.down_m_s  = weight * fst.down_m_s  + (1 - weight) * other.down_m_s;
  retNED.yaw_deg   = weight * fst.yaw_deg   + (1 - weight) * other.yaw_deg;
  return retNED;
}
  
vector<Offboard::VelocityNEDYaw> get_reasonable_actions(std::vector<Offboard::VelocityNEDYaw> base_actions) {
  // NOTE: this is PoC only -- should really MILP this stuff; for higher dimensions can just try random values?
  //  assert(base_actions.size() == 2);
    
  vector<Offboard::VelocityNEDYaw> reasonable_actions;

  cout << "Starting actions: ";
  for (auto i = base_actions.begin(); i != base_actions.end(); ++i) {
    string s = "[" + to_string(i->north_m_s) + ", " + to_string(i->east_m_s) + ", " + to_string(i->down_m_s) + "]";
    std::cout << s << endl;
  }

  /* Not necessary, but helps with rounding issues */
  droneutil::scaleToUnitVector(base_actions[0]);
  droneutil::scaleToUnitVector(base_actions[1]);

  auto action_range = get_action_range(base_actions);

  /* How many numbers do we care about in a range [0,1] */
  unsigned int precision = droneutil::RANDOM_SEARCH_GRANULARITY;
  
  unsigned int num_actions =
    precision * fabsf(action_range.first.north_m_s - action_range.second.north_m_s) *
    precision * fabsf(action_range.first.east_m_s  - action_range.second.east_m_s ) *
    precision * fabsf(action_range.first.down_m_s  - action_range.second.down_m_s );
    
    
  // Explore the space between the conflicting actions
  Offboard::VelocityNEDYaw new_action;
  for(unsigned long long i = 0; i <= num_actions; i++) {
    new_action = get_action_in_range(action_range);

    droneutil::scaleToMaxVelocity(new_action);

    reasonable_actions.push_back(new_action);
  }

  /*
  cout << "Reasonable actions: ";
  
  for (auto i = reasonable_actions.begin(); i != reasonable_actions.end(); ++i) {
    string s = "[" + to_string(i->north_m_s) + ", " + to_string(i->east_m_s) + ", " + to_string(i->down_m_s) + "]";
    std::cout << s << endl;
  }
  */
  
  return reasonable_actions;
}

  
/* Returns the optimal action */
Offboard::VelocityNEDYaw get_optimal_action(const std::vector<StlExpr*>& properties,
					    const std::vector<float>& weights,
					    const std::vector<Offboard::VelocityNEDYaw>& conflicting_actions,
					    std::shared_ptr<StateStore> store,
					    int t) {
  assert(properties.size() == weights.size() && properties.size());

  vector<Offboard::VelocityNEDYaw> potential_actions;

  // Just use conflicted actions if we're not synthesizing
  if(droneutil::SYNTHESIZE_ACTIONS) {
    potential_actions = get_reasonable_actions(conflicting_actions);    
    // Concatenate enforcer conflicted actions to proposed actions
    potential_actions.insert(potential_actions.end(), conflicting_actions.begin(), conflicting_actions.end());
  } else {
    potential_actions = conflicting_actions;
  }

  double max_global_rob = 0;
  Offboard::VelocityNEDYaw max_action;
  bool is_first = true;
  
  cout << "-------------------------------Robustness: " << endl;
  Signal est_signal = *store->getSignal();
  
  for(auto cur_action : potential_actions) {
    float cur_global_rob = 0;
    
    // Sum weighted robustness values for each property at time `t+1`
    // Time t+1 because that includes the estimated signal
    for(int i = 0; i < properties.size(); i++) {
      get_est_signal(store->getSignal(), cur_action, &est_signal);
      float robustness = properties[i]->robustness(&est_signal, t+1);
      // We want to reuse our est_signal, so we have to pop off the last element
      est_signal.pop();
      
      cur_global_rob += weights[i] * robustness;

      /*
      string action_str = "[" + to_string(cur_action.north_m_s) + ", " + to_string(cur_action.east_m_s) + ", " + to_string(cur_action.down_m_s) + "]";
      cout << action_str << ": R## "<< to_string(weights[i]) << " * " << to_string(robustness) << endl;
      
      cout << "Estimated signal: " << to_string(new_pos_north) << ", " << to_string(new_pos_east) << ", " << to_string(new_pos_down) << ", [";
      cout <<  to_string(action.north_m_s) << ", " <<  to_string(action.east_m_s) << ", " << to_string(action.down_m_s) << "]" << endl;
      
      std::cout << "Est signal (part 2): " <<
      est_signal.value("pos_north_m") << "," <<
      est_signal.value("pos_east_m") << "," <<
      est_signal.value("pos_down_m") << ",[" <<
      est_signal.value("vel_north_m_s") << "," <<
      est_signal.value("vel_east_m_s") << "," <<
      est_signal.value("vel_down_m_s") << "]" << std::endl <<
      "Enemey: " <<
      est_signal.value("enemy_pos_north_m") << "," <<
      est_signal.value("enemy_pos_east_m") << "," <<
      est_signal.value("enemy_pos_down_m") << ",[" <<
      est_signal.value("enemy_vel_north_m_s") << "," <<
      est_signal.value("enemy_vel_east_m_s") << "," <<
      est_signal.value("enemy_vel_down_m_s") << "]" << std::endl;
      */
    }
    /*
    string s = "[" + to_string(cur_action.north_m_s) + ", " + to_string(cur_action.east_m_s) + ", " + to_string(cur_action.down_m_s) + "]";
    std::cout << " R## " << s << " : " << to_string(cur_global_rob) << endl;
    */
    
    // Update values if new max
    if(cur_global_rob > max_global_rob || is_first) {
      max_global_rob = cur_global_rob;
      max_action = cur_action;
      is_first = false;
    }
  }

  return max_action;
}

namespace cdra {

  RobustnessCoordinator::RobustnessCoordinator(
					       std::shared_ptr<dronecode_sdk::Offboard> offboard,
					       std::shared_ptr<dronecode_sdk::Telemetry> telemetry,
					       std::shared_ptr<StateStore> store)
    : Coordinator(offboard, telemetry, store) {}

  RobustnessCoordinator::~RobustnessCoordinator() {}

  void RobustnessCoordinator::addEnforcer(std::shared_ptr<cdra::Enforcer> e, float weight){
    Coordinator::addEnforcer(e);
    StlEnforcer* se = (StlEnforcer*)e.get();
    weights.insert({se, weight});
  }

  void RobustnessCoordinator::sendVelocityNed(const dronecode_sdk::Offboard::VelocityNEDYaw &velocity_ned_yaw){

    Offboard::VelocityNEDYaw newNED;

    cout << "##### Performing enforcer coordination" << endl;

    vector<Offboard::VelocityNEDYaw> intersection;
    // An "active" enforcer is one whose property is violated and thus is "activated" to take some action
    vector<StlEnforcer*> activeEnforcers;
    vector<StlExpr*> properties;
    vector<float> prop_weights;
    vector<Offboard::VelocityNEDYaw> actions;
    
    for (int i=0; i < enforcers.size(); i++){
      StlEnforcer* enforcer = (StlEnforcer*)(enforcers.at(i)).get();

      // We only care about enforcers whose properties are violated
      // NOTE: this is optimizing global robustness wrt only properties in conflict (not global ones)
      if (!enforcer->checkProp()) {
	activeEnforcers.push_back(enforcer);
	properties.push_back((StlExpr*)(enforcer->getProp()));
	prop_weights.push_back(weights[enforcer]);
	auto enforcer_action = enforcer->enforce(velocity_ned_yaw);

	// Concatenate enforcer actions to actions
	actions.insert(actions.end(), enforcer_action.begin(), enforcer_action.end());
      }
    }    

    if (activeEnforcers.empty()){
      // No properties are violated, so just pass off the given velocity vector to offboard
      newNED = velocity_ned_yaw;
    } else if (activeEnforcers.size() < 2){
      // Only one enforcer is activated, so no conflict to resolve
      cout << "### One enforcer activated: " << activeEnforcers.at(0)->getName() << endl;

      // Send the action that is closest to the original action
      if(droneutil::CHOOSE_LEAST_DIFFERENT_ACTION) {
	newNED = get_least_different(velocity_ned_yaw, actions);
      } else { // Just use the first action
	newNED = actions[0];
      }
    } else {
      int numActiveEnforcers = activeEnforcers.size();
      cout << "### Mutliple enforcers activated: " << numActiveEnforcers << endl;
      
      newNED = get_optimal_action(properties, prop_weights, actions, store, activeEnforcers.at(0)->getTime());

      if(!droneutil::SUGGEST_ACTION_RANGE) {
	assert(activeEnforcers.size() == actions.size());
      }
      /* If you want to see how similar the synthesized actions are to proposed actions
      if(droneutil::SYNTHESIZE_ACTIONS) {
	float max_sim = droneutil::cosineSimilarity(actions[0], newNED);
	for(auto a : actions) {
	  float cur_sim = droneutil::cosineSimilarity(a, newNED);
	  if(cur_sim > max_sim) {
	    max_sim = cur_sim;
	  }	  
	}
	cout << "synthesized action has %f cosine similarity to a suggested action: " << max_sim << endl;
	if(max_sim > 0.96) {
	  printf("using action that didn't really need to be synthesized.\n");
	}
      }
      */
    }

    cout << "Final coordinated action (velocity): (" << newNED.north_m_s << "," << newNED.east_m_s << "," << newNED.down_m_s << "), " << endl;

    newNED.yaw_deg = velocity_ned_yaw.yaw_deg;
    Coordinator::sendVelocityNed(newNED);
  }
}
