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

#include "ElasticStlEnforcer.h"
#include <algorithm>
#include <iostream>
#include <math.h>

using namespace dronecode_sdk;
using namespace std;

namespace cdra {

ElasticStlEnforcer::ElasticStlEnforcer(std::shared_ptr<dronecode_sdk::Offboard> offboard,
			 std::shared_ptr<dronecode_sdk::Telemetry> telemetry,
			 std::shared_ptr<StateStore> store)
	: Enforcer(offboard, telemetry, store)
{
	enforcerName = "Elastic STL Enforcer";
	ttiFun = new TTIFun(lowerx, upperx, lowery, uppery, lowerz, upperz, safeThreshold);
	// Property 1: Current TTI is above safe threshold
	propTTISafe = new Prop(ttiFun);
	// Property 2: TTI has not been below safe threshold for MAX_UNSAFE_PERIOD
	propTTISafePast = new Implies(new Not(new Prop(ttiFun)),			 
				      new Not(new PastGlobal(new Not(new Prop(ttiFun)),
							     MAX_UNSAFE_PERIOD, 1)));
}

ElasticStlEnforcer::~ElasticStlEnforcer() {
  delete ttiFun;
  delete propTTISafe;
  delete propTTISafePast;
}


//-- check if out of bounds
bool ElasticStlEnforcer::unsafe(const Telemetry::PositionNED &loc)
{
  return (loc.north_m <= lowerx || loc.north_m >= upperx || loc.east_m <= lowery ||
          loc.east_m >= uppery || loc.down_m <= lowerz || loc.down_m >= upperz);
}

void ElasticStlEnforcer::weightedMergeCommands(dronecode_sdk::Offboard::VelocityNEDYaw& msg,
		const dronecode_sdk::Offboard::VelocityNEDYaw& other, float weight) {
	msg.north_m_s = weight * msg.north_m_s + (1 - weight) * other.north_m_s;
	msg.east_m_s = weight * msg.east_m_s + (1 - weight) * other.east_m_s;
	msg.down_m_s = weight * msg.down_m_s + (1 - weight) * other.down_m_s;
}

float ElasticStlEnforcer::computeAlpha(float tti) {
	float weight;
    weight = (tti < 0.0) ? 0.0 : (tti < safeThreshold ? tti / safeThreshold : 1);    
    /*
    float ratio = tti / safeThreshold;    
    if (tti > safeThreshold) weight = 1.0;
    else if (tti < 0.0 || ratio < 0.5) {
      weight = 0.0;
    } else {
      weight = ratio - 0.5;
    }
    */
    return weight;
}

vector<Offboard::VelocityNEDYaw> ElasticStlEnforcer::enforce(
		const Offboard::VelocityNEDYaw &velNEDin) {
	bool enforced = false;

	Offboard::VelocityNEDYaw velNED = velNEDin; // make a writable copy
	Telemetry::PositionVelocityNED posvel = telemetry->position_velocity_ned();
	Telemetry::PositionNED& position = posvel.position;
	//Telemetry::VelocityNED& velocity = posvel.velocity;

	Signal* signal = store->getSignal();
	int tick = store->currTick();

	// Check whether properties are satisfied
	bool propTTISafeSat = propTTISafe->sat(signal, tick);
	bool propTTISafePastSat = propTTISafePast->sat(signal, tick);
	cout << endl;
	cout << "Lastest signal: " <<
	  signal->value("pos_east_m") << "," <<
	  signal->value("pos_north_m") << "," <<
	  signal->value("pos_down_m") << "," <<	  
	  signal->value("vel_east_m_s") << "," <<	  
	  signal->value("vel_north_m_s") << "," <<	  
	  signal->value("vel_down_m_s") << endl;
	cout << "Signal length: " << signal->length() << endl;
	cout << "TTI: " << ttiFun->value(signal) + safeThreshold << endl;
		
	if (propTTISafeSat) {
	  cout << "Property SAT: " << propTTISafe->exprStr() << " (robustness = " << propTTISafe->robustness(signal, tick) << ")" << endl;
	} else {
	  cout << "************************" << endl;
	  cout << "Property VIOLATED: " << propTTISafe->exprStr() << " (robustness = " << propTTISafe->robustness(signal, tick) << ")" << endl;
	  if (!propTTISafePastSat) {
	    cout << "Property VIOLATED: " << propTTISafePast->exprStr()  << " (robustness = " << propTTISafePast->robustness(signal, tick) << ")" << endl;
	  }
	  cout << "************************" << endl;
	}

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

	if (!propTTISafeSat) {
	  if (!propTTISafePastSat) {
	    // TTI has been below a safe threshold for too long
	    // Move towards the origin immediately
	    weightedMergeCommands(velNED, toOrigin, 0.0);
	    cout << "Enforce -> go to origin" << endl;
	  } else {
	    // Current TTI is below a safe threshold
	    // Gradually move towards the origin
	    float alpha = computeAlpha(ttiFun->value(signal) + safeThreshold);		
	    weightedMergeCommands(velNED, toOrigin, alpha);
	  }
	  cout << "Enforced tti=" << ttiFun->value(signal) << endl;
	  enforced = true;
	}

	if (enforced) {
		cout << "Enforced: in vel in=" << velNEDin.north_m_s << ',' << velNEDin.east_m_s << ',' << velNEDin.down_m_s
				<< " vel out=" << velNED.north_m_s << ',' << velNED.east_m_s << ',' << velNED.down_m_s
				<< endl;
	}
    return Enforcer::enforce(velNED);
}

} /* namespace cdra */
