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

#include "StlEnforcer.h"
#include "StlExpr.h"
#include <iostream>

namespace cdra {

    StlEnforcer::StlEnforcer(
            std::shared_ptr<dronecode_sdk::Offboard> offboard,
            std::shared_ptr<dronecode_sdk::Telemetry> telemetry,
            std::shared_ptr<StateStore> store) : Enforcer(offboard, telemetry, store){
    }

    StlEnforcer::~StlEnforcer(){
        delete prop;
    }

    /**
     * @return true iff the property is satisfied by the current signal
     */
    bool StlEnforcer::checkProp(){
        Signal* signal = store->getSignal();
        int tick = store->currTick();
	std::cout << getName() << " robustness: " << prop->robustness(signal, tick) << std::endl;
        return prop->sat(signal, tick);
    }

    /**
     * @return the property associated with this enforcer
     */
    StlExpr* StlEnforcer::getProp(){
      return prop;
    }

    int StlEnforcer::getTime(){
      return store->currTick();
    }

    /**
     * Generic STL-based enforcement loop
     * Check whether the property is satisfied by the current signal and
     * generate appropriate actions to be performed
     * @param velocity_ned_yaw: Input velocity vector
     * @return return the set of actions to be performed, as suggested by this enforcer
     */
    std::vector<dronecode_sdk::Offboard::VelocityNEDYaw> StlEnforcer::enforce(
            const dronecode_sdk::Offboard::VelocityNEDYaw &velocity_ned_yaw){
        if (checkProp()){
            return actPropSatisfied(velocity_ned_yaw);
        } else {
            return actPropViolated(velocity_ned_yaw);
        }
    }

}

