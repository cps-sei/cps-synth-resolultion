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

#include "IntersectingCoordinator.h"
#include "StlEnforcer.h"
#include "DroneUtil.h"
#include <iostream>

using namespace dronecode_sdk;
using namespace std;

namespace cdra {

    IntersectingCoordinator::IntersectingCoordinator(
            std::shared_ptr<dronecode_sdk::Offboard> offboard,
            std::shared_ptr<dronecode_sdk::Telemetry> telemetry) : Coordinator(offboard, telemetry){}

    IntersectingCoordinator::~IntersectingCoordinator() {}

    void IntersectingCoordinator::sendVelocityNed(const dronecode_sdk::Offboard::VelocityNEDYaw &velocity_ned_yaw){

        Offboard::VelocityNEDYaw newNED;

        cout << "##### Performing enforcer coordination" << endl;

        vector<Offboard::VelocityNEDYaw> intersection;
        vector<StlEnforcer*> activeEnforcers;
        for (int i=0; i < enforcers.size(); i++){
            StlEnforcer* enforcer = (StlEnforcer*)(enforcers.at(i)).get();
            if (!enforcer->checkProp())
                activeEnforcers.push_back(enforcer);
        }

        if (activeEnforcers.empty()){
            newNED = velocity_ned_yaw;
        } else if (activeEnforcers.size() < 2){
            cout << "### One enforcer activated: " << activeEnforcers.at(0)->getName() << endl;
            vector<Offboard::VelocityNEDYaw> cmds = activeEnforcers.at(0)->enforce(velocity_ned_yaw);
            newNED = cmds.at(0);
        } else {
            cout << "### Mutliple enforcers activated" << endl;
            for (int i=0; i < activeEnforcers.size() - 1; i++){
                auto actionSet1 = activeEnforcers.at(i)->enforce(velocity_ned_yaw);
                auto actionSet2 = activeEnforcers.at(i+1)->enforce(velocity_ned_yaw);

                for (auto a1 : actionSet1){
                    for (auto a2 : actionSet2){
		      if (droneutil::cmp(a1, a2, epsilon)) intersection.push_back(a1);
                    }
                }
            }
            if (intersection.empty()) {
                cout << "NO INTERSECTING ACTIONS!" << endl;
                exit (EXIT_FAILURE);
            }

            newNED = intersection.at(0);
        }

        Coordinator::sendVelocityNed(newNED);
    }
}
