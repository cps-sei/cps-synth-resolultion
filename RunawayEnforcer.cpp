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

#include "RunawayEnforcer.h"
#include <iostream>
#include <math.h>

using namespace dronecode_sdk;
using namespace std;

#define PI 3.14159265

namespace cdra {

    RunawayEnforcer::RunawayEnforcer(std::shared_ptr<dronecode_sdk::Offboard> offboard,
        std::shared_ptr<dronecode_sdk::Telemetry> telemetry,
        std::shared_ptr<StateStore> store) : StlEnforcer(offboard, telemetry, store) {
        enforcerName = "Runaway Enforcer";

	// Property: Current DTT is maintained above a safe threshold
        dttFun = new DTTFun(droneutil::ENEMY_CHASE_DISTANCE);

	prop = new Prop(dttFun);
        //prop = new PastGlobal(new Prop(dttFun), droneutil::TICKS_TO_CORRECT, 0);
    }

    RunawayEnforcer::~RunawayEnforcer(){
        delete dttFun;
    }

    std::vector<dronecode_sdk::Offboard::VelocityNEDYaw>
            RunawayEnforcer::actPropSatisfied(const dronecode_sdk::Offboard::VelocityNEDYaw &velocity_ned_yaw)  {
        std::vector<dronecode_sdk::Offboard::VelocityNEDYaw> newNEDs;
        newNEDs.push_back(velocity_ned_yaw);
        return newNEDs;
    }

    std::vector<dronecode_sdk::Offboard::VelocityNEDYaw>
            RunawayEnforcer::actPropViolated(const dronecode_sdk::Offboard::VelocityNEDYaw &velocity_ned_yaw) {

        /**
         * Goal: Direct the drone in directions away from the enemy drone
         */
        Signal* signal = store->getSignal();

        cout << endl;
        cout << "*** Runaway Enforcer: Property violated!" << endl;

        // retrieve the velocity vector of the trailing drone
        float enemy_pos_north = signal->value("enemy_pos_north_m");
        float enemy_pos_east  = signal->value("enemy_pos_east_m" );
        float enemy_pos_down  = signal->value("enemy_pos_down_m" );

	std::vector<dronecode_sdk::Offboard::VelocityNEDYaw> newNEDs;
	// Compute the velocity vector from the enemy to the ego drone
	// and use this vector with MAX_DRONE_SPEED as the new vector for the ego drone
	// i.e., fly the ego drone in the same direction as the enemy drone but with the max speed
        Offboard::VelocityNEDYaw enemyNED = droneutil::computeNEDtoTarget(enemy_pos_north, enemy_pos_east, enemy_pos_down,
                signal->value("pos_north_m"),signal->value("pos_east_m"),signal->value("pos_down_m"),
                droneutil::MAX_DRONE_SPEED, velocity_ned_yaw.yaw_deg);

	if(!droneutil::EGO_Z_VELOCITY) { enemyNED.down_m_s = 0; }
	cout << "Runaway vector (" << enemyNED.north_m_s << "," << enemyNED.east_m_s << "," << enemyNED.down_m_s << ")" << endl;

	enemyNED.yaw_deg = velocity_ned_yaw.yaw_deg;
        newNEDs.push_back(enemyNED);

	if(droneutil::SUGGEST_ACTION_RANGE) {
	  // Determine how much we can deviate from the best runaway vector
	  double acceptable_deviation = max((droneutil::MAX_DRONE_SPEED - droneutil::ENEMY_DRONE_SPEED) / droneutil::MAX_DRONE_SPEED, 0.0f);
	  double sqrt_dev = sqrt(acceptable_deviation);
	  double angle_deviation = acceptable_deviation/2*M_PI;
	
	
	  /* Want the cone-ish shape of vectors that deviate from our vector by angle_deviation 
	   * I don't know how to do that (and don't have time to figure out rn), so I'll just use this instead: 
	   */
	  int num_intervals = 5;
	  for(float i = -sqrt_dev; i <= sqrt_dev; i += sqrt_dev*2/num_intervals) {
	    for(float j = -sqrt_dev; j <= sqrt_dev; j += sqrt_dev*2/num_intervals) {
	      for(float k = -sqrt_dev; k <= sqrt_dev; k += sqrt_dev*2/num_intervals) {
		Offboard::VelocityNEDYaw curNED {
		  i+enemyNED.north_m_s,
		    j+enemyNED.east_m_s,
		    k+enemyNED.down_m_s,
		    velocity_ned_yaw.yaw_deg
		    };
		droneutil::scaleToMaxVelocity(curNED);
		newNEDs.push_back(curNED);
	      }
	    }
	  }
	}
        return newNEDs;
    }

}
