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

#include "StateStore.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include "DroneUtil.h"
#include "DTTFun.h"
#include "TTIFun.h"
#include "StlExpr.h"
#include "DTGFun.h"
#include "json/json/json.h"

using namespace std;

namespace cdra {

    StateStore::StateStore(std::shared_ptr<dronecode_sdk::Telemetry> telemetry,
			   std::shared_ptr<EnemyDrone> enemyDrone) :
      telemetry(telemetry), enemyDrone(enemyDrone) {
        tick = 0;
        signal = new Signal(signalNames);
    }

    StateStore::~StateStore(){
        delete signal;
    }

    void StateStore::recordNewState(){
        // Store the latest signal values
        tick++;
        // obtain the state of the drone under control
        dronecode_sdk::Telemetry::PositionVelocityNED posvel = telemetry->position_velocity_ned();
        dronecode_sdk::Telemetry::PositionNED& position = posvel.position;
        dronecode_sdk::Telemetry::VelocityNED& velocity = posvel.velocity;

        // generate the simulated state of the enemy drone
        dronecode_sdk::Telemetry::PositionVelocityNED enemyPosvel = enemyDrone->position_velocity_ned();
        dronecode_sdk::Telemetry::PositionNED& enemyPos = enemyPosvel.position;
        dronecode_sdk::Telemetry::VelocityNED& enemyVel = enemyPosvel.velocity;


        std::vector<float> currSignalVal {
                position.east_m  , position.north_m  , position.down_m  ,
                velocity.east_m_s, velocity.north_m_s, velocity.down_m_s,
                enemyPos.east_m  , enemyPos.north_m  , enemyPos.down_m  ,
                enemyVel.east_m_s, enemyVel.north_m_s, enemyVel.down_m_s,
        };
	
        signal->append(currSignalVal);

	// for debugging
        std::cout << std::endl;
        std::cout << "Latest signal vel: " <<
            signal->value("pos_north_m") << "," <<
             signal->value("pos_east_m") << "," <<
             signal->value("pos_down_m") << ",[" <<
             signal->value("vel_north_m_s") << "," <<
             signal->value("vel_east_m_s") << "," <<
             signal->value("vel_down_m_s") << "]" << std::endl <<
             "Enemey: " <<
             signal->value("enemy_pos_north_m") << "," <<
             signal->value("enemy_pos_east_m") << "," <<
             signal->value("enemy_pos_down_m") << ",[" <<
             signal->value("enemy_vel_north_m_s") << "," <<
             signal->value("enemy_vel_east_m_s") << "," <<
             signal->value("enemy_vel_down_m_s") << "]" << std::endl;
        std::cout << "Signal length: " << signal->length() << std::endl;
    }

    int StateStore::currTick() {
        return tick;
    }

    void StateStore::addStlExpr(StlExpr* stlExpr) {
      stlExprs.push_back(stlExpr);
    }
  
    Signal* StateStore::getSignal(){
        return signal;
    }
  
    void StateStore::writeAnimationData(std::string logdir) {
      writeSignal({"pos_east_m", "pos_north_m", "pos_down_m"}, logdir + "plot_animation_data_ego.dat");
      writeSignal({"enemy_pos_east_m", "enemy_pos_north_m", "enemy_pos_down_m"}, logdir + "plot_animation_data_enemy.dat");
    }
  
    void StateStore::writeSignal(std::vector<std::string> names, std::string fname){
        std::ofstream myfile;
        myfile.open (fname);
        myfile << "curve" << std::endl;
	
        for (int t=1; t < signal->length(); t++){
	  // write the part of signal for each of the given names to fname	  
	  for (int j=0; j < names.size(); j++){
                myfile << signal->value(names.at(j),t) << ",";
                if (j == names.size() - 1){
                    myfile << t << std::endl;
                }
	  }
	}
	myfile.close();
    }

    void StateStore::writeChasePoints(std::string fname){
        std::ofstream myfile;
        myfile.open (fname);
        myfile << "scatter" << std::endl;

        for (int t=1; t < signal->length(); t++){
            const float delta_east  = signal->value("pos_east_m", t) - signal->value("enemy_pos_east_m", t);
            const float delta_north = signal->value("pos_north_m",t) - signal->value("enemy_pos_north_m",t);
            const float delta_down  = signal->value("pos_down_m", t) - signal->value("enemy_pos_down_m", t);
            const float delta = sqrt(pow(delta_east , 2.0) +
				     pow(delta_north, 2.0) +
				     pow(delta_down , 2.0));
	    // If the delta between the enemy drone and ego done is less than a certain distance,
	    // record this location and tick 
            if (delta < droneutil::ENEMY_CHASE_DISTANCE*0.5){
                myfile << signal->value("pos_east_m",t) << ","
		       << signal->value("pos_north_m",t) << ","
		       << t << /*-signal->value("pos_down_m", t) <<*/ std::endl;
            }
        }
        myfile.close();
    }
  
  /* Write all the locations where coordination happened? */
  void StateStore::writeCoordinatedPoints(std::string fname){
    std::ofstream myfile;
    myfile.open (fname);
    myfile << "scatter" << std::endl;
    
    for (int t=1; t < signal->length(); t++){

      // Count active enforcers
      unsigned int active_enforcers = 0;
      for(int i = 0; i < stlExprs.size(); i++) {
	if(!stlExprs[i]->sat(signal, t)) {
	    active_enforcers++;
	}
      }
      
      // If >=2 enforcers are active, record this location&tick
      if (active_enforcers >= 2) {
	myfile << signal->value("pos_east_m" , t) << ","
	       << signal->value("pos_north_m", t) << ","
	       << t << std::endl;
      }
    }
    
    myfile.close();
  }
  void StateStore::writeCoordinatorActivity(std::string dirname) {    
    Json::Value coordinator_data;
    Json::Value run_data;
    
    // array per stlExpr
    for(auto stlExpr : stlExprs) {
      Json::Value vec(Json::arrayValue);
      vec.append(vec);
      vec[0] = Json::arrayValue; // This prevents a null entry as the first value, I guess
      coordinator_data[stlExpr->generalStr().c_str()]["all_robustness_values"] = vec;
      run_data["coordinators_active"] = vec;
    }

    int active_enforcers = 0;
    // append all robustness values for each tick
    for(int t = 1; t < signal->length(); t++) {
      active_enforcers = 0;
      for(int i = 0; i < stlExprs.size(); i++) {
	if(!stlExprs[i]->sat(signal,t)) {
	  active_enforcers++;
	}

	coordinator_data[stlExprs[i]->generalStr().c_str()]["all_robustness_values"][0].append(stlExprs[i]->robustness(signal,t));
      }
      run_data["coordinators_active"][0].append(active_enforcers);
    }
    
    int num_coordinated_ticks = 0;
    for(auto num_coordinators : run_data["coordinators_active"]) {
      if(num_coordinators >= 2) {
	num_coordinated_ticks++;
      }
    }
    {
      Json::Value vec(Json::arrayValue);
      vec.append(num_coordinated_ticks);
      run_data["num_coordinated_ticks"] = vec;
    }
    {
      // Note: coordinators_active should have 1 entry for each tick (i.e., its length should equal the total number of ticks)
      Json::Value vec(Json::arrayValue);
      vec.append(run_data["coordinators_active"].size());
      run_data["total_num_ticks"] = vec;
    }
    std::ofstream coordinator_activity_file;
    std::ofstream run_data_file;
    coordinator_activity_file.open(dirname+"coordinator_activity.json");
    coordinator_activity_file << coordinator_data << endl;
    coordinator_activity_file.close();
    run_data_file.open(dirname+"run_data.json");
    run_data_file << run_data << endl;
    run_data_file.close();
  }
  
  void StateStore::writeJSONData(std::string fname) {
    std::ofstream myfile;
    myfile.open(fname);
    Json::Value data;

    int ticks_violated;
    int cur_violation;
    int max_violation;
    // Determine max number of ticks a given property was violated
    for(int i = 0; i < stlExprs.size(); i++) {
      ticks_violated = 0; // Total number of ticks in violation
      cur_violation  = 0; // Current ticks in a row of violation
      max_violation  = 0; // Max number of ticks in a row of violation
      float min_robustness = 0;
      for(int t = 1; t < signal->length(); t++) {
	if(!stlExprs[i]->sat(signal, t)) {
	  ticks_violated++;
	  cur_violation++;
	  if(cur_violation > max_violation) {
	    max_violation = cur_violation;
	  }
	  if(stlExprs[i]->robustness(signal, t) < min_robustness) {
	    min_robustness = stlExprs[i]->robustness(signal,t);
	  }
	} else {
	  cur_violation = 0;
	}
      }
      Json::Value vec(Json::arrayValue);
      vec.append(ticks_violated);
      data[stlExprs[i]->generalStr().c_str()]["total_ticks_violated"] = vec;
      vec = Json::Value(Json::arrayValue);
      vec.append(max_violation);
      data[stlExprs[i]->generalStr().c_str()]["max_violation_length"] = vec;
      vec = Json::Value(Json::arrayValue);
      vec.append(min_robustness);
      data[stlExprs[i]->generalStr().c_str()]["min_robustness"] = vec;
    }

    // actual max distance outside boundary
    float max_dist_outside_boundary = 0;
    max_violation = 0;
    cur_violation = 0;
    ticks_violated = 0;
    for(int t = 1; t < signal->length(); t++) {
      float max_dimension = max({
	  fabsf(signal->value("pos_north_m")),
	  fabsf(signal->value("pos_east_m")),
	  fabsf(signal->value("pos_down_m"))
	    });
      // Assumes all boundary sides are same len
      if(max_dimension >= droneutil::BOUNDARY_X_MAX && max_dimension > max_dist_outside_boundary) {
	max_dist_outside_boundary = max_dimension - droneutil::BOUNDARY_X_MAX;
	cur_violation++;
	ticks_violated++;
	if(cur_violation > max_violation) {
	  max_violation = cur_violation;
	}
      } else {
	cur_violation = 0;
      }
    }
    Json::Value vec(Json::arrayValue);
    vec = Json::Value(Json::arrayValue);
    vec.append(max_dist_outside_boundary);
    data["Boundary"]["max_distance_outside_boundary"] = vec;
    vec = Json::Value(Json::arrayValue);
    vec.append(ticks_violated);
    data["Boundary"]["total_ticks_outside_boundary"] = vec;
    vec = Json::Value(Json::arrayValue);
    vec.append(max_violation);
    data["Boundary"]["max_ticks_outside_boundary_contiguous"] = vec;

    unsigned int num_catches = 0;
    for(int t = 1; t < signal->length(); t++) {
      float deltaX = signal->value("pos_north_m") - signal->value("enemy_pos_north_m");
      float deltaY = signal->value("pos_east_m" ) - signal->value("enemy_pos_east_m" );
      float deltaZ = signal->value("pos_down_m" ) - signal->value("enemy_pos_down_m" );

      double diag = sqrt(pow(deltaX, 2) +
			 pow(deltaY, 2) + pow(deltaZ, 2));
      if(diag <= droneutil::CATCH_DISTANCE) {
	num_catches++;
      }
    }
    vec = Json::Value(Json::arrayValue);
    vec.append(num_catches);
    data["Runaway"]["num_catches"] = vec;

    myfile << data << endl;
  }
}
