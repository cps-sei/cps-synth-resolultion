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

#include <iostream>
#include <vector>

#include "PriorityCoordinator.h"
#include "StlEnforcer.h"

using namespace dronecode_sdk;
using namespace std;

namespace cdra {

  void PriorityCoordinator::addEnforcer(std::shared_ptr<cdra::Enforcer> e, float weight) {
    Coordinator::addEnforcer(e);
    StlEnforcer* se = (StlEnforcer*)e.get();
    weights.insert({se, weight});
  }
  
  PriorityCoordinator::PriorityCoordinator(
					   std::shared_ptr<dronecode_sdk::Offboard> offboard,
					   std::shared_ptr<dronecode_sdk::Telemetry> telemetry)
    : Coordinator(offboard, telemetry){}

  PriorityCoordinator::~PriorityCoordinator() {}

  StlEnforcer* PriorityCoordinator::getPriorityEnforcer(vector<StlEnforcer*> activeEnforcers) {
    if(activeEnforcers.size() == 0) {
      return nullptr;
    }
    auto argmax = activeEnforcers[0];
    auto max = weights[argmax];
    for(auto enforcer : activeEnforcers) {
      if(weights[enforcer] > max) {
	argmax = enforcer;
	max = weights[enforcer];
      }
    }
    return argmax;
  }
  
  void PriorityCoordinator::sendVelocityNed(const dronecode_sdk::Offboard::VelocityNEDYaw &velocity_ned_yaw){

    // Find how many active enforcers
    vector<StlEnforcer*> activeEnforcers;

    Offboard::VelocityNEDYaw newNED;
    
    for(int i = 0; i < enforcers.size(); i++) {
      StlEnforcer* enforcer = (StlEnforcer*)(enforcers.at(i)).get();
      if(!enforcer->checkProp()) {
	activeEnforcers.push_back(enforcer);
      }
    }

    if(activeEnforcers.empty()) {
      newNED = velocity_ned_yaw;
    } else if(activeEnforcers.size() == 1) {
      cout << "### One enforcer activated: " << activeEnforcers.at(0)->getName() << endl;

      // Use the first proposed action of the enforcer
      newNED = activeEnforcers[0]->enforce(velocity_ned_yaw)[0];
    } else {
      cout << "### Multiple enforcers activated: " << activeEnforcers.size() << endl;
      auto priority_enforcer = getPriorityEnforcer(activeEnforcers);

      // Use the first proposed action of the enforcer
      newNED = priority_enforcer->enforce(velocity_ned_yaw)[0];
    }
    
    Coordinator::sendVelocityNed(newNED);
  }
}
