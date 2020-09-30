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

#include "Coordinator.h"
#include "DroneUtil.h"

namespace cdra {

  Coordinator::Coordinator(std::shared_ptr<dronecode_sdk::Offboard> offboard,
			   std::shared_ptr<dronecode_sdk::Telemetry> telemetry)
    : offboard(offboard), telemetry(telemetry) {}
  
  Coordinator::Coordinator(std::shared_ptr<dronecode_sdk::Offboard> offboard,
			   std::shared_ptr<dronecode_sdk::Telemetry> telemetry,
			   std::shared_ptr<StateStore> store)
    : offboard(offboard), telemetry(telemetry), store(store) {}

  Coordinator::~Coordinator() {}

  void Coordinator::addEnforcer(std::shared_ptr<cdra::Enforcer> e, float weight) {
    enforcers.push_back(e);
  }

  std::vector<std::shared_ptr<cdra::Enforcer>> Coordinator::getEnforcers() {
    return enforcers;
  }
  
  void Coordinator::sendVelocityNed(const dronecode_sdk::Offboard::VelocityNEDYaw &velocity_ned_yaw) {

    // Zero out z-velocity if desired
    if(!droneutil::EGO_Z_VELOCITY) {
      auto tmp = velocity_ned_yaw;
      tmp.down_m_s = 0;
      offboard->send_velocity_ned(tmp);
    }
    // Base coordinator does not do anything; simply forwards the velocity command to offboard
    offboard->send_velocity_ned(velocity_ned_yaw);
  }
}
