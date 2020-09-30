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


#include "WeightedCoordinator.h"
#include "StlEnforcer.h"
#include "DroneUtil.h"
#include <iostream>

using namespace dronecode_sdk;
using namespace std;

namespace cdra {

    WeightedCoordinator::WeightedCoordinator(
            std::shared_ptr<dronecode_sdk::Offboard> offboard,
            std::shared_ptr<dronecode_sdk::Telemetry> telemetry)
      : Coordinator(offboard, telemetry){}

    WeightedCoordinator::~WeightedCoordinator() {}

    void WeightedCoordinator::addEnforcer(std::shared_ptr<cdra::Enforcer> e, float weight){
        Coordinator::addEnforcer(e);
        StlEnforcer* se = (StlEnforcer*)e.get();
        weights.insert({se, weight});
    }

    void WeightedCoordinator::sendVelocityNed(const dronecode_sdk::Offboard::VelocityNEDYaw &velocity_ned_yaw){

        Offboard::VelocityNEDYaw newNED;

        cout << "##### Performing enforcer coordination" << endl;

        vector<Offboard::VelocityNEDYaw> intersection;
	// An "active" enforcer is one whose property is violated and thus is "activated" to take some action
        vector<StlEnforcer*> activeEnforcers;
	double unused_weight = 0;
        for (int i=0; i < enforcers.size(); i++){
            StlEnforcer* enforcer = (StlEnforcer*)(enforcers.at(i)).get();
	    // We only care about enforcers whose properties are violated	   
            if (!enforcer->checkProp()) {
                activeEnforcers.push_back(enforcer);
	    }
	    // Keep track of weights that won't be used
	    else {
	      unused_weight += weights[enforcer];
	    }
        }

        if (activeEnforcers.empty()){
	    // No properties are violated, so just pass off the given velocity vector to offboard
            newNED = velocity_ned_yaw;
        } else if (activeEnforcers.size() < 2) {
	    // Only one enfrocer is activated, so no conflict to resolve
            cout << "### One enforcer activated: " << activeEnforcers.at(0)->getName() << endl;
            vector<Offboard::VelocityNEDYaw> actions = activeEnforcers.at(0)->enforce(velocity_ned_yaw);
            newNED = actions.at(0);
        } else {
            int numActiveEnforcers = activeEnforcers.size();
            cout << "### Mutliple enforcers activated: " << numActiveEnforcers << endl;
            for (int i=0; i < numActiveEnforcers; i++){
                vector<Offboard::VelocityNEDYaw> actions = activeEnforcers.at(i)->enforce(velocity_ned_yaw);
                cout << actions.size() << " actions available from " << activeEnforcers.at(i)->getName() << endl;
		// Each enforcer, in principle, could produce multiple actions
		// But for now, we assume there's only one enforced action
		// i.e., take the first one in the list
                Offboard::VelocityNEDYaw action = actions.at(0);

		// Redistribute weights to be equal to 1 again
                float w = weights[activeEnforcers.at(i)] / (1 - unused_weight);
		
                if (i == 0) {
		    // "initialize" the new velocity vector with the weighted version of
		    // the one produced from the first enforcer		  
                    newNED.north_m_s = action.north_m_s*w;
                    newNED.east_m_s = action.east_m_s*w;
                    newNED.down_m_s = action.down_m_s*w;
                    newNED.yaw_deg = action.yaw_deg*w;
                } else {
                    newNED.north_m_s = newNED.north_m_s + action.north_m_s*w;
                    newNED.east_m_s = newNED.east_m_s + action.east_m_s*w;
                    newNED.down_m_s = newNED.down_m_s + action.down_m_s*w;
                    newNED.yaw_deg = newNED.yaw_deg + action.yaw_deg*w;
                }
            }
        }


        cout << "Final coordinated action: (" << newNED.north_m_s << "," << newNED.east_m_s << "), " << endl;
        Coordinator::sendVelocityNed(newNED);
    }
}
