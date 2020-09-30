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

#include "Enforcer.h"
#include "DroneUtil.h"
#include <cmath>
#include <dronecode_sdk/dronecode_sdk.h>
#include <chrono>
#include <set>

using namespace dronecode_sdk;
using std::chrono::steady_clock;

namespace cdra {

Enforcer::Enforcer(std::shared_ptr<dronecode_sdk::Offboard> offboard,
		std::shared_ptr<dronecode_sdk::Telemetry> telemetry,
		std::shared_ptr<StateStore> store)
	: offboard(offboard), telemetry(telemetry), store(store), enforcerName("Null Enforcer")
{ }

Enforcer::~Enforcer() {
	// TODO Auto-generated destructor stub
}

const char* Enforcer::getName() const {
	return enforcerName.c_str();
}

void Enforcer::send(const dronecode_sdk::Offboard::VelocityNEDYaw& velocity_ned_yaw) {
	offboard->send_velocity_ned(velocity_ned_yaw);
}

std::vector<dronecode_sdk::Offboard::VelocityNEDYaw> Enforcer::enforce(
		const dronecode_sdk::Offboard::VelocityNEDYaw &velocity_ned_yaw) {

	std::vector<dronecode_sdk::Offboard::VelocityNEDYaw> newNEDs;
	newNEDs.push_back(velocity_ned_yaw);
	return newNEDs;
}

} /* namespace cdra */
