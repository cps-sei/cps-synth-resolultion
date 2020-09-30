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

#ifndef MISSIONAPP_BOUNDARYENFORCER_H
#define MISSIONAPP_BOUNDARYENFORCER_H

#include "StlEnforcer.h"
#include "TTIFun.h"

namespace cdra {

    class BoundaryEnforcer : public StlEnforcer {

        //const int MAX_UNSAFE_PERIOD=4;
        TTIFun* ttiFun;
        //-- the cube boundaries are specified as lower and upper bounds
        //-- on X, Y and Z
        //-- In NED x -> north, y -> east, z -> down
	//-- (in meters)
        float lowerx = droneutil::BOUNDARY_X_MIN, upperx = droneutil::BOUNDARY_X_MAX,
	      lowery = droneutil::BOUNDARY_Y_MIN, uppery = droneutil::BOUNDARY_Y_MAX,
	      lowerz = droneutil::BOUNDARY_Z_MIN, upperz = droneutil::BOUNDARY_Z_MAX;
        float safeThreshold = droneutil::BOUNDARY_SAFE_TTI_THRESHOLD; // seconds

        //float computeTTI(const dronecode_sdk::Telemetry::PositionVelocityNED &loc) const;
        void weightedMergeCommands(dronecode_sdk::Offboard::VelocityNEDYaw& msg,
                                   const dronecode_sdk::Offboard::VelocityNEDYaw& other, float weight);

        void enumerateSafeVelNEDs(dronecode_sdk::Offboard::VelocityNEDYaw velNED,
                std::vector<dronecode_sdk::Offboard::VelocityNEDYaw> &newNEDs);
        dronecode_sdk::Offboard::VelocityNEDYaw makeNED(float north_m_s, float east_m_s, float down_m_s, float yaw_deg);


    protected:
        std::vector<dronecode_sdk::Offboard::VelocityNEDYaw>
	  actPropSatisfied(const dronecode_sdk::Offboard::VelocityNEDYaw &velocity_ned_yaw);
        std::vector<dronecode_sdk::Offboard::VelocityNEDYaw>
	  actPropViolated (const dronecode_sdk::Offboard::VelocityNEDYaw &velocity_ned_yaw);

    public:
        BoundaryEnforcer(std::shared_ptr<dronecode_sdk::Offboard> offboard,
                        std::shared_ptr<dronecode_sdk::Telemetry> telemetry,
                        std::shared_ptr<StateStore> store);
        virtual ~BoundaryEnforcer();
    };

}


#endif //MISSIONAPP_BOUNDARYENFORCER_H
