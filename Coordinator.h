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

#ifndef MISSIONAPP_COORDINATOR_H
#define MISSIONAPP_COORDINATOR_H

#include <dronecode_sdk/offboard.h>
#include <dronecode_sdk/telemetry.h>
#include <vector>
#include "Enforcer.h"
#include "StateStore.h"

namespace cdra{

    /**
     * * Coordinator
     * *
     * Given a velocity command from a (auto-)pilot,
     * a coordinator processes possible actions on the command by a set of enforcers
     * and generates a final velocityNED to be sent to the offboard control.
     *
     * The base coordinator simply forwards the input velocity command to offboard.
     * */
    class Coordinator {

    protected:
        std::shared_ptr<dronecode_sdk::Offboard> offboard;
        std::shared_ptr<dronecode_sdk::Telemetry> telemetry;
	std::shared_ptr<StateStore> store;
        std::vector<std::shared_ptr<cdra::Enforcer>> enforcers;    // set of enforcers being coordinated
	
    public:

        Coordinator(std::shared_ptr<dronecode_sdk::Offboard> offboard,
                    std::shared_ptr<dronecode_sdk::Telemetry> telemetry,
		    std::shared_ptr<StateStore> store);
	Coordinator(std::shared_ptr<dronecode_sdk::Offboard> offboard,
                    std::shared_ptr<dronecode_sdk::Telemetry> telemetry);
        virtual ~Coordinator();

        virtual void addEnforcer(std::shared_ptr<cdra::Enforcer> e, float weight=0);
	std::vector<std::shared_ptr<cdra::Enforcer>> getEnforcers();
	// to be implemented by the coordinator class that extends this
        virtual void sendVelocityNed(const dronecode_sdk::Offboard::VelocityNEDYaw& velocity_ned_yaw);

    };
}



#endif //MISSIONAPP_COORDINATOR_H
