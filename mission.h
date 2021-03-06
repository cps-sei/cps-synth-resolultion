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

#ifndef MISSIONAPP_MISSION_H
#define MISSIONAPP_MISSION_H

#include <dronecode_sdk/offboard.h>
#include <dronecode_sdk/telemetry.h>
#include <dronecode_sdk/action.h>

#include "Coordinator.h"

namespace cdra {
  class Mission {
  protected:
    std::shared_ptr<dronecode_sdk::Offboard> offboard;
    std::shared_ptr<dronecode_sdk::Telemetry> telemetry;
    std::shared_ptr<dronecode_sdk::Action> action;
    std::shared_ptr<cdra::StateStore> store;
    cdra::Coordinator* coordinator;
    
  public:
    
    Mission(std::shared_ptr<dronecode_sdk::Offboard> offboard,
	    std::shared_ptr<dronecode_sdk::Telemetry> telemetry,
	    std::shared_ptr<dronecode_sdk::Action> action,
	    std::shared_ptr<cdra::StateStore> store,
	    cdra::Coordinator* coordinator);
    
    
    bool setup();
    virtual bool run() = 0;
    bool cleanup();
    bool log(std::string logdir=".");
  };
}
#endif //MISSIONAPP_MISSION_H
