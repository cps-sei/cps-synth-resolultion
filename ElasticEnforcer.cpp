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

#include "ElasticEnforcer.h"
#include <algorithm>
#include <iostream>
#include <vector>
#include <math.h>

using namespace dronecode_sdk;
using namespace std;

namespace cdra {

ElasticEnforcer::ElasticEnforcer(std::shared_ptr<dronecode_sdk::Offboard> offboard,
		std::shared_ptr<dronecode_sdk::Telemetry> telemetry,
		std::shared_ptr<StateStore> store)
	: Enforcer(offboard, telemetry, store)
{
	enforcerName = "Elastic Enforcer";
}

ElasticEnforcer::~ElasticEnforcer() {
	// TODO Auto-generated destructor stub
}


//-- check if out of bounds
bool ElasticEnforcer::unsafe(const Telemetry::PositionNED &loc)
{
  return (loc.north_m <= lowerx || loc.north_m >= upperx || loc.east_m <= lowery ||
          loc.east_m >= uppery || loc.down_m <= lowerz || loc.down_m >= upperz);
}

float ElasticEnforcer::computeTTI(const Telemetry::PositionVelocityNED &loc) const
{
  float res = 1000.0;

  if(loc.velocity.north_m_s < 0) res = std::min(res, fabsf(lowerx - loc.position.north_m) / (-loc.velocity.north_m_s));
  if(loc.velocity.north_m_s > 0) res = std::min(res, fabsf(upperx - loc.position.north_m) / (loc.velocity.north_m_s));

  if(loc.velocity.east_m_s < 0) res = std::min(res, fabsf(lowery - loc.position.east_m) / (-loc.velocity.east_m_s));
  if(loc.velocity.east_m_s > 0) res = std::min(res, fabsf(uppery - loc.position.east_m) / (loc.velocity.east_m_s));

  if(loc.velocity.down_m_s < 0) res = std::min(res, fabsf(lowerz - loc.position.down_m) / (-loc.velocity.down_m_s));
  if(loc.velocity.down_m_s > 0) res = std::min(res, fabsf(upperz - loc.position.down_m) / (loc.velocity.down_m_s));

  return res;
}

void ElasticEnforcer::weightedMergeCommands(dronecode_sdk::Offboard::VelocityNEDYaw& msg,
		const dronecode_sdk::Offboard::VelocityNEDYaw& other, float weight) {
	msg.north_m_s = weight * msg.north_m_s + (1 - weight) * other.north_m_s;
	msg.east_m_s = weight * msg.east_m_s + (1 - weight) * other.east_m_s;
	msg.down_m_s = weight * msg.down_m_s + (1 - weight) * other.down_m_s;
}


std::vector<dronecode_sdk::Offboard::VelocityNEDYaw> ElasticEnforcer::enforce(
		const Offboard::VelocityNEDYaw &velNEDin) {
	bool enforced = false;

	Offboard::VelocityNEDYaw velNED = velNEDin; // make a writable copy
	Telemetry::PositionVelocityNED posvel = telemetry->position_velocity_ned();
	Telemetry::PositionNED& position = posvel.position;

	// pre-compute go to origin msg
	float const maxSpeed = 2;
	float diag = sqrt(pow(position.north_m, 2) + pow(position.east_m, 2));
	Offboard::VelocityNEDYaw toOrigin = velNED;
	toOrigin.north_m_s = (-position.north_m / diag) * maxSpeed;
	toOrigin.east_m_s = (-position.east_m / diag) * maxSpeed;
	toOrigin.down_m_s = 0;

	cout << "Current position: x=" << position.north_m <<  " y=" << position.east_m 
         << " z=" << position.down_m << endl;

    if (unsafe(position)){
    	cout << "CRASH!" << endl;
    }

	if (state == State::NORMAL) {
		//-- check if out of bounds
		if (unsafe(position)) {
			hoverCount = 50;
			state = State::HOVER;
			cout << "unsage->HOVER" << endl;
		} else {

			//-- compute time to intercept, i.e., estimated time to cross a
			//-- boundary.
			float tti = computeTTI(posvel);
			float alpha = (tti < 0.0) ? 0.0 : (tti < 2.0 ? tti / 2.0 : 1);
			if (alpha < 1.0) {
				weightedMergeCommands(velNED, toOrigin, alpha);
				cout << "Enforced tti alpha=" << alpha << endl;
				enforced = true;
			}
		}
	}

	//-- if hover state, keep hovering till timeout
	if (state == State::HOVER) {
		velNED.north_m_s = 0;
		velNED.east_m_s = 0;
		velNED.down_m_s = 0;
		enforced = true;
		cout << "Hover" << endl;
		hoverCount--;
		if (hoverCount == 0) {
			state = State::ENFORCE;
			cout << "HOVER->ENFORCE" << endl;
		}
	}

	//-- if enforcement state. this means we are definitely out of
	//-- bounds. this should not happen frequently.
	if (state == State::ENFORCE) {
		//-- if we are back in the tether, stop enforcing.
		if (!unsafe(position)) {
			state = State::NORMAL;
			enforced = false;
			cout << "ENFORCE->NORMAL" << endl;
		} else {
			if (position.down_m >= upperz) {
				velNED.north_m_s = 0;
				velNED.east_m_s = 0;
				velNED.down_m_s = -2;
			} else if (position.down_m <= lowerz) {
				velNED.north_m_s = 0;
				velNED.east_m_s = 0;
				velNED.down_m_s = 2;
			} else { //-- otherwise move toward the origin
				weightedMergeCommands(velNED, toOrigin, 0.0);
				cout << "Enforce -> go to origin" << endl;
			}
			enforced = true;
		}
	}
	if (enforced) {
		cout << "Enforced: in vel in=" << velNEDin.north_m_s << ',' << velNEDin.east_m_s << ',' << velNEDin.down_m_s
				<< " vel out=" << velNED.north_m_s << ',' << velNED.east_m_s << ',' << velNED.down_m_s
				<< endl;
	}
	return Enforcer::enforce(velNED);
}

} /* namespace cdra */
