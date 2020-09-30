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

#ifndef MISSIONAPP_ENEMYDRONE_H
#define MISSIONAPP_ENEMYDRONE_H

#include "DroneModel.h"
#include <dronecode_sdk/action.h>
#include <dronecode_sdk/telemetry.h>
#include <dronecode_sdk/dronecode_sdk.h>

namespace cdra {

    /**
     * Enemy drone
     */
    class EnemyDrone {
      /*
      // Base UUID for drones
      const unsigned long long base_uuid = 5283920058631409230;

      // How many drones have come before this one?
      static unsigned long long drone_num = 1;
      
      // Actual UUID
      const unsigned long long uuid = base_uuid + drone_num;
      
      const unsigned long long uuid = 5283920058631409231;

      dronecode_sdk::DronecodeSDK dc;
      */
      
      std::shared_ptr<dronecode_sdk::Telemetry> telemetry;
      std::shared_ptr<dronecode_sdk::Telemetry> ego_telemetry;
      std::shared_ptr<dronecode_sdk::Action> action;
      
    public:

      EnemyDrone(dronecode_sdk::System& system, std::shared_ptr<dronecode_sdk::Telemetry> ego_telemetry);
      ~EnemyDrone();
	
      // Returns the most recent position of the drone
      dronecode_sdk::Telemetry::PositionVelocityNED position_velocity_ned();
      
      // Kill the enemy drone
      void kill();
    };
}

#endif //MISSIONAPP_ENEMYDRONE_H
