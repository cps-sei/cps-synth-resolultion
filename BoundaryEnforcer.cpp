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

#include "BoundaryEnforcer.h"
#include "DroneUtil.h"
#include <iostream>
#include <math.h>

using namespace dronecode_sdk;
using namespace std;

namespace cdra {

    BoundaryEnforcer::BoundaryEnforcer(std::shared_ptr<dronecode_sdk::Offboard> offboard,
				       std::shared_ptr<dronecode_sdk::Telemetry> telemetry,
				       std::shared_ptr<StateStore> store) : StlEnforcer(offboard, telemetry, store) {
        enforcerName = "Boundary Enforcer";
        ttiFun = new TTIFun(lowerx, upperx, lowery, uppery, lowerz, upperz, safeThreshold);
        prop = new Prop(ttiFun);
	//prop = new PastGlobal(new Prop(ttiFun), droneutil::TICKS_TO_CORRECT, 0);
    }

    BoundaryEnforcer::~BoundaryEnforcer(){
        delete ttiFun;
    }

    std::vector<dronecode_sdk::Offboard::VelocityNEDYaw>
    BoundaryEnforcer::actPropSatisfied(const dronecode_sdk::Offboard::VelocityNEDYaw &velocity_ned_yaw) {
        std::vector<dronecode_sdk::Offboard::VelocityNEDYaw> newNEDs;
        newNEDs.push_back(velocity_ned_yaw);
        return newNEDs;
    }

    std::vector<dronecode_sdk::Offboard::VelocityNEDYaw>
    BoundaryEnforcer::actPropViolated(const dronecode_sdk::Offboard::VelocityNEDYaw &velocity_ned_yaw) {

        Offboard::VelocityNEDYaw velNED = velocity_ned_yaw; // make a writable copy
        Telemetry::PositionVelocityNED posvel = telemetry->position_velocity_ned();
        Telemetry::PositionNED& position = posvel.position;

        Signal* signal = store->getSignal();

        cout << endl;
        cout << "*** Boundary Enforcer: Property violated!" << endl;
        cout << "Latest signal: " <<
            signal->value("pos_north_m") << "," <<
            signal->value("pos_east_m") << "," <<
            signal->value("pos_down_m") << ",[" <<
            signal->value("vel_north_m_s") << "," <<
            signal->value("vel_east_m_s") << "," <<
            signal->value("vel_down_m_s") << "]" << endl;
        //cout << "Signal length: " << signal->length() << endl;
        //cout << "TTI: " << ttiFun->value(signal) + safeThreshold << endl;

        // pre-compute go to origin msg
	
        float diag;

	if(droneutil::EGO_Z_VELOCITY) {
	  diag = sqrt(pow(position.north_m, 2) +
		      pow(position.east_m, 2 ) +
		      pow(position.down_m-2.5, 2)); //-2.5 down position is the starting height after takeoff
	} else {
	  diag = sqrt(pow(position.north_m, 2) +
		      pow(position.east_m, 2 ));
	}

	if(!droneutil::EGO_Z_VELOCITY) {
	  velNED.down_m_s = 0;
	}
	
        Offboard::VelocityNEDYaw toOrigin = velNED;
        toOrigin.north_m_s = (-position.north_m / diag) * droneutil::MAX_DRONE_SPEED;
        toOrigin.east_m_s  = (-position.east_m  / diag) * droneutil::MAX_DRONE_SPEED;
	if(droneutil::EGO_Z_VELOCITY) {
	  toOrigin.down_m_s  = (-position.down_m  / diag) * droneutil::MAX_DRONE_SPEED;
	}
	
	// This is basically what the original "ElasticEnforcer" does
	// Compute the velocity vector to the origin & merge it with 
        weightedMergeCommands(velNED, toOrigin, 0.0);
        cout << "To origin vector (" << velNED.north_m_s << "," << velNED.east_m_s << ")" << endl;


        // generate new enforcement commands
        std::vector<dronecode_sdk::Offboard::VelocityNEDYaw> newNEDs;

        
	if(droneutil::SUGGEST_ACTION_RANGE) {
	  // Another approach:
	  // Instead of a single vector, generate a range of vectors that move the drone away from the boundary
	  enumerateSafeVelNEDs(velocity_ned_yaw, newNEDs);
	  for (int i=0; i < newNEDs.size(); i++) {
            auto x = newNEDs.at(i);
            std::cout << "(" << x.north_m_s << "," << x.east_m_s << "), ";
	  }
	  std::cout << std::endl;
	}
        newNEDs.push_back(velNED);
	
        return newNEDs;
    }

    Offboard::VelocityNEDYaw BoundaryEnforcer::makeNED(float north_m_s, float east_m_s, float down_m_s, float yaw_deg) {
        Offboard::VelocityNEDYaw newNED;
        newNED.east_m_s  = east_m_s;
        newNED.north_m_s = north_m_s;
        newNED.down_m_s  = droneutil::EGO_Z_VELOCITY ? down_m_s : 0;
        newNED.yaw_deg   = yaw_deg;
        return newNED;
    }

    void BoundaryEnforcer::enumerateSafeVelNEDs(Offboard::VelocityNEDYaw velNED, std::vector<Offboard::VelocityNEDYaw> &newNEDs){
        Signal* sig = store->getSignal();
        const float pos_east_m  = sig->value("pos_east_m");
        const float pos_north_m = sig->value("pos_north_m");
        const float pos_down_m  = droneutil::EGO_Z_VELOCITY ? sig->value("pos_down_m") : 0;
        const float vel_east    = sig->value("vel_east_m_s");
        const float vel_north   = sig->value("vel_north_m_s");
        const float vel_down    = sig->value("vel_down_m_s");
        const float yaw = velNED.yaw_deg;

	float diag;
	if(droneutil::EGO_Z_VELOCITY) {
	  diag = sqrt(pow(pos_north_m, 2) + pow(pos_east_m, 2) + pow(pos_down_m, 2));
	} else {
	  diag = sqrt(pow(pos_north_m, 2) + pow(pos_east_m, 2));
	}
	
        float to_origin_north_m_s = (-pos_north_m / diag) * droneutil::MAX_DRONE_SPEED;
        float to_origin_east_m_s  = (-pos_east_m  / diag) * droneutil::MAX_DRONE_SPEED;
	float to_origin_down_m_s  = droneutil::EGO_Z_VELOCITY ? (-pos_down_m  / diag) * droneutil::MAX_DRONE_SPEED : 0;
	
	// closeToX means the ego drone is approaching the boundary on the x-axis
        bool closeToX = ttiFun->closeToXBoundary(pos_east_m, pos_north_m, pos_down_m, vel_east, vel_north, vel_down);
	// closeToY means the ego drone is approaching the boundary on the y-axis
        bool closeToY = ttiFun->closeToYBoundary(pos_east_m, pos_north_m, pos_down_m, vel_east, vel_north, vel_down);
	// closeToY means the ego drone is approaching the boundary on the z-axis (-down = up)
	bool closeToZ = ttiFun->closeToZBoundary(-pos_down_m, -vel_down);

	
	// If the drone is approaching the boundary on both axes (i.e., corner), then fly it toward the origin
        if (closeToX && closeToY && closeToZ) {
            newNEDs.push_back(makeNED(to_origin_north_m_s, to_origin_east_m_s, to_origin_down_m_s, yaw));
            return;
        }

	if (closeToX && closeToY) {
	  newNEDs.push_back(makeNED(to_origin_north_m_s, to_origin_east_m_s, 0, yaw));
	  return;
        }

	if (closeToX && closeToZ) {
	  newNEDs.push_back(makeNED(to_origin_north_m_s, 0, to_origin_down_m_s, yaw));
	  return;
        }
	
	if (closeToY && closeToZ) {
	  newNEDs.push_back(makeNED(0, to_origin_east_m_s, to_origin_down_m_s, yaw));
	  return;
        }

	// If the drone is approaching the boundary on x-axis, fly the drone towards various points along x=0 
        if (closeToX) {
            for (int i=lowery + 1; i < uppery; i++){
	      for(int j=lowerz + 1; j < upperz; j++) {
                newNEDs.push_back(
                        droneutil::computeNEDtoTarget(
                                pos_north_m, pos_east_m, pos_down_m,
                                0, i, -j,
                                droneutil::MAX_DRONE_SPEED, yaw));
	      }
            }
            return;
        }

	// If the drone is approaching the boundary on y-axis, fly the drone towards various points along y=0 
        if (closeToY) {
            for (int i=lowerx + 1; i < upperx; i++){
	      for(int j=lowerz + 1; j < upperz; j++) {
                newNEDs.push_back(
                        droneutil::computeNEDtoTarget(
                                pos_north_m, pos_east_m, pos_down_m,
                                i, 0, -j,
                                droneutil::MAX_DRONE_SPEED, yaw));
	      }
            return;
	    }
        }

	// If the drone is approaching the boundary on y-axis, fly the drone towards various points along z = -2
        if (closeToZ) {
            for (int i=lowerx + 1; i < upperx; i++){
	      for(int j=lowery + 1; j < uppery; j++) {
                newNEDs.push_back(
                        droneutil::computeNEDtoTarget(
                                pos_north_m, pos_east_m, pos_down_m,
                                i, j, -2,
                                droneutil::MAX_DRONE_SPEED, yaw));
	      }
            }
            return;
        }
    }

    void BoundaryEnforcer::weightedMergeCommands(dronecode_sdk::Offboard::VelocityNEDYaw& msg,
        const dronecode_sdk::Offboard::VelocityNEDYaw& other, float weight) {
        msg.north_m_s = weight * msg.north_m_s + (1 - weight) * other.north_m_s;
        msg.east_m_s  = weight * msg.east_m_s  + (1 - weight) * other.east_m_s;
        msg.down_m_s  = weight * msg.down_m_s  + (1 - weight) * other.down_m_s;
    }
}
