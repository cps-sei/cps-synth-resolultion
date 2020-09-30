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

#ifndef ELASTICENFORCER_H_
#define ELASTICENFORCER_H_

#include "Enforcer.h"

namespace cdra {


/**
 * Enforcer that pulls the drone to the center of a cube
 *
 * This is based on the original CubeEnforcerJPH.
 * As the drone gets close to the boundaries, the pull towards the center
 * increases being inversely to the time to intercept the boundary
 */
class ElasticEnforcer: public Enforcer {
public:
	ElasticEnforcer(std::shared_ptr<dronecode_sdk::Offboard> offboard,
            std::shared_ptr<dronecode_sdk::Telemetry> telemetry,
            std::shared_ptr<StateStore> store);

	virtual ~ElasticEnforcer();

    std::vector<dronecode_sdk::Offboard::VelocityNEDYaw> enforce(
			const dronecode_sdk::Offboard::VelocityNEDYaw &velocity_ned_yaw) override;

    //-- the cube boundaries are specified as lower and upper bounds
    //-- on X, Y and Z
	//-- In NED x -> north, y -> east, z -> down
    float lowerx = -10, upperx = 10, lowery = -10, uppery = 10, lowerz = -10, upperz = -1;

protected:
	bool unsafe(const dronecode_sdk::Telemetry::PositionNED &loc);
	float computeTTI(const dronecode_sdk::Telemetry::PositionVelocityNED &loc) const;
	void weightedMergeCommands(dronecode_sdk::Offboard::VelocityNEDYaw& msg,
			const dronecode_sdk::Offboard::VelocityNEDYaw& other, float weight);

	enum class State { NORMAL, ENFORCE, HOVER };
	State state = State::NORMAL;

    //-- countdown for HOVER state
    int hoverCount = 0;
};

} /* namespace cdra */

#endif /* ELASTICENFORCER_H_ */
