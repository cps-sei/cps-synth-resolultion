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

#include "EnemyDrone.h"
#include "DroneUtil.h"
#include <cmath>
#include <iostream>
#include <dronecode_sdk/action.h>
#include <dronecode_sdk/telemetry.h>
#include <dronecode_sdk/dronecode_sdk.h>
#include <assert.h>

namespace cdra {

  EnemyDrone::EnemyDrone(dronecode_sdk::System& system, std::shared_ptr<dronecode_sdk::Telemetry> ego_telemetry) : ego_telemetry(ego_telemetry) {
    telemetry = std::make_shared<dronecode_sdk::Telemetry>(system);
    action  = std::make_shared<dronecode_sdk::Action>(system);
  }
  
  EnemyDrone::~EnemyDrone() {}

  void EnemyDrone::kill() {
    action->kill();
  }
  
  dronecode_sdk::Telemetry::PositionVelocityNED EnemyDrone::position_velocity_ned() {

    auto relative = telemetry->position_velocity_ned();
    auto origin = ego_telemetry->home_position();
    auto actual = relative;

    actual.position = droneutil::pos2ned(telemetry->position(), origin);
    
    // Fix z pos
    actual.position.down_m = relative.position.down_m;
    
    // Make sure that enemy telemetry is good
    assert(!isnan(actual.position.north_m));

    return actual;
  }
}
