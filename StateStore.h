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

#ifndef MISSIONAPP_DRONESTATE_H
#define MISSIONAPP_DRONESTATE_H

#include <dronecode_sdk/telemetry.h>
#include "Signal.h"
#include "EnemyDrone.h"
#include "StlExpr.h"
#include "DroneUtil.h"

namespace cdra {

    /**
     * Stores system states to be accessed globally
     * In particular, it stores the global clock (tick) and
     * the signal up to the tick.
     */
    class StateStore {

        int tick;
        Signal* signal;
        std::shared_ptr<dronecode_sdk::Telemetry> telemetry;
        std::shared_ptr<EnemyDrone> enemyDrone;
        std::vector<std::string> signalNames {
                "pos_east_m", "pos_north_m", "pos_down_m",
                "vel_east_m_s", "vel_north_m_s", "vel_down_m_s",
                "enemy_pos_east_m", "enemy_pos_north_m", "enemy_pos_down_m",
                "enemy_vel_east_m_s", "enemy_vel_north_m_s", "enemy_vel_down_m_s"};
	std::vector<StlExpr*> stlExprs;

    public:
        StateStore(std::shared_ptr<dronecode_sdk::Telemetry> telemetry,
		   std::shared_ptr<EnemyDrone> enemyDrone);
        ~StateStore();
        // Records the latest state from the telemetry
        void recordNewState();
        // Returns the current tick
        int currTick();
        // Returns the current signal
        Signal* getSignal();
	
	// Add StlExpr being used
	void addStlExpr(StlExpr* stlExpr);
	
	// for logging
	// Write the signal projected over "names" to file "fname"
        void writeSignal(std::vector<std::string> names, std::string fname);
	void writeAnimationData(std::string logdir);
	// Mark points in tick where the enemy drone has "caught up" to the ego drone
        void writeChasePoints(std::string fname);
	void writeCoordinatedPoints(std::string fname);
	void writeCoordinatorActivity(std::string fname);
	void writeJSONData(std::string fname);
    };
}

#endif //MISSIONAPP_DRONESTATE_H
