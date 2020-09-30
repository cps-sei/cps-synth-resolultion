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

#ifndef MISSIONAPP_MISSILEENFORCER_H
#define MISSIONAPP_MISSILEENFORCER_H

#include "StlEnforcer.h"
#include "StlExpr.h"
#include "DroneUtil.h"
#include "ReconFun.h"

namespace cdra {

    class MissileEnforcer : public StlEnforcer {

      // Just reuse ReconFun -- basically go to a specific altitude when in an XY region
      ReconFun* missileFun;

      // XY region in which to avoid missiles by raising altitude
      float lowerx = -10, upperx = -3, lowery = -10, uppery = -3;

      // Height needed to perform recon mission
      float missileElevation = 7.0;

      // Acceptable threshold
      float acceptableThreshold = 1.0;
	
    protected:
        std::vector<dronecode_sdk::Offboard::VelocityNEDYaw>
            actPropSatisfied(const dronecode_sdk::Offboard::VelocityNEDYaw &velocity_ned_yaw);
        std::vector<dronecode_sdk::Offboard::VelocityNEDYaw>
            actPropViolated(const dronecode_sdk::Offboard::VelocityNEDYaw &velocity_ned_yaw);

    public:
        MissileEnforcer(std::shared_ptr<dronecode_sdk::Offboard> offboard,
            std::shared_ptr<dronecode_sdk::Telemetry> telemetry,
            std::shared_ptr<StateStore> store);
        virtual ~MissileEnforcer();
    };

}

#endif //MISSIONAPP_MISSILEENFORCER_H
