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

#include "FlightEnforcer.h"
#include "DroneUtil.h"

#include <iostream>
#include <math.h>
#include <cmath>

#include "DroneUtil.h"

using namespace dronecode_sdk;
using namespace std;

namespace cdra {

    FlightEnforcer::FlightEnforcer(std::shared_ptr<dronecode_sdk::Offboard> offboard,
        std::shared_ptr<dronecode_sdk::Telemetry> telemetry,
        std::shared_ptr<StateStore> store) : StlEnforcer(offboard, telemetry, store) {
        enforcerName = "Flight Enforcer";
	
        // Property: Current DTG is maintained above a safe threshold
        dtgFun = new DTGFun(1);
	prop   = new Prop(dtgFun);
    }

    FlightEnforcer::~FlightEnforcer(){
        delete dtgFun;
    }

    std::vector<dronecode_sdk::Offboard::VelocityNEDYaw>
            FlightEnforcer::actPropSatisfied(const dronecode_sdk::Offboard::VelocityNEDYaw &velocity_ned_yaw)  {
        std::vector<dronecode_sdk::Offboard::VelocityNEDYaw> newNEDs;
        newNEDs.push_back(velocity_ned_yaw);
        return newNEDs;
    }

    std::vector<dronecode_sdk::Offboard::VelocityNEDYaw>
            FlightEnforcer::actPropViolated(const dronecode_sdk::Offboard::VelocityNEDYaw &velocity_ned_yaw) {
      /* Just ignore this prop if we don't want z-velocity */
      if(!droneutil::EGO_Z_VELOCITY) {
	return actPropSatisfied(velocity_ned_yaw);
      }
      
        /**
         * Goal: Direct the drone in directions away from the enemy drone
         */

        cout << endl;
        cout << "*** Flight Enforcer: Property violated!" << endl;

        std::vector<dronecode_sdk::Offboard::VelocityNEDYaw> newNEDs;

	float down_vel = -droneutil::MAX_DRONE_SPEED;
	
        Offboard::VelocityNEDYaw newNED{
	  0,
	  0,
	  down_vel,
	  velocity_ned_yaw.yaw_deg
	};

	if(droneutil::SUGGEST_ACTION_RANGE) {
	  /* Provide num_intervals^2 more actions */
	  int num_intervals = 5;
	  for(float i = -droneutil::MAX_DRONE_SPEED; i < droneutil::MAX_DRONE_SPEED; i+= droneutil::MAX_DRONE_SPEED/num_intervals) {
	    for(float j = -droneutil::MAX_DRONE_SPEED; j < droneutil::MAX_DRONE_SPEED; j+= droneutil::MAX_DRONE_SPEED/num_intervals) {
	      Offboard::VelocityNEDYaw curNED { i, j, down_vel, velocity_ned_yaw.yaw_deg };
	      droneutil::scaleToMaxVelocity(curNED);
	      newNEDs.push_back(curNED);
	    }
	  }
	}
        newNEDs.push_back(newNED);

        return newNEDs;
    }

}
