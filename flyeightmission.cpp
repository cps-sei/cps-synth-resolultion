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

#include <chrono>
#include <iostream>
#include <memory>
#include <cmath>
#include <thread>
#include <stdbool.h>

#include "mission.h"
#include "flyeightmission.h"
#include "DroneUtil.h"

using namespace dronecode_sdk;
using namespace std;

using hrclock = std::chrono::high_resolution_clock;

namespace cdra {
  FlyEightMission::FlyEightMission(std::shared_ptr<dronecode_sdk::Offboard> offboard,
				   std::shared_ptr<dronecode_sdk::Telemetry> telemetry,
				   std::shared_ptr<dronecode_sdk::Action> action,
				   std::shared_ptr<cdra::StateStore> store,
				   cdra::Coordinator* coordinator)
    : Mission(offboard, telemetry, action, store, coordinator){ }
  
  bool FlyEightMission::run() {
    const float angularStep = -0.1;
    const float figureLength = 20;
    float theta = 2.5 * M_PI;

    double waypointX = 0;
    double waypointY = 0;

    bool firstWpt = true;

    while (theta > 0.5 * M_PI) {

      float prevWaypointX = waypointX;
      float prevWaypointY = waypointY;
      double scale = 2 / (3 - cos(2 * theta));
      waypointX = figureLength * scale * cos(theta);
      waypointY = figureLength * scale * sin(2 * theta) / 2;

      theta += angularStep;

      if (firstWpt) {
	firstWpt = false;
	continue;
      }

      //    	std::cout << "next waypoint: " << waypointX << ',' << waypointY << std::endl;

      double distance = sqrt(pow(waypointX - prevWaypointX, 2) + pow(waypointY - prevWaypointY, 2));
      double estimatedTimeToWpt = distance / droneutil::MAX_DRONE_SPEED;
      auto startTime = hrclock::now();

      // Main fly-8 loop
      do {
	// is that point behind?

	// Record the latest state info. in the global state storage
	store->recordNewState();
	    
	Telemetry::PositionVelocityNED posvel = telemetry->position_velocity_ned();
	float posX = posvel.position.north_m;
	float posY = posvel.position.east_m;
	float yaw = telemetry->attitude_euler_angle().yaw_deg; // 0-north clockwise

	double deltaX = waypointX - posX;
	double deltaY = waypointY - posY;
	//            std::cout << "pos: " << posX << ',' << posY
	//            		<< " deltaPos: " << deltaX << ',' << deltaY
	//					<< std::endl;

	double nextYaw= droneutil::getAngle(deltaX, deltaY);
	double deltaYaw = fmod((nextYaw > yaw) ? nextYaw - yaw : yaw - nextYaw, 360.0);
	if (deltaYaw > 180) {
	  deltaYaw = 360 - deltaYaw;
	}

	if (deltaYaw > 90) { // the next waypoint is behind
	  std::cout << "flew past waypoint" << std::endl;
	  //                break;
	}

	// compute velocity vector components
	double diag = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
	double velX = droneutil::MAX_DRONE_SPEED * deltaX / diag;
	double velY = droneutil::MAX_DRONE_SPEED * deltaY / diag;

	double figureYaw = droneutil::getAngle(waypointX - prevWaypointX, waypointY - prevWaypointY);
	//            std::cout << "set_velocity_ned " << (float)velX << ',' << (float)velY
	//            		<< ',' << figureYaw << std::endl;
	    
	auto start_time = chrono::high_resolution_clock::now();
	coordinator->sendVelocityNed({(float) velX, (float) velY, 0.0f, (float) figureYaw});	    
	auto end_time = chrono::high_resolution_clock::now();

	auto time_taken = chrono::duration_cast<chrono::milliseconds>(end_time-start_time).count();
	cout << "Coordinator took "
	     << time_taken << " milliseconds.\n" << endl; 
	    

	// Sleep for TICK_DURATION minus however long the last computation took
	//   (or 0 if the computation took longer than the tick duration)
	// i.e., total duration between ticks is close to the actual TICK_DURATION
	auto sleep_time = max((int)(std::lround(droneutil::TICK_DURATION*1000)-time_taken), 0);
	if(sleep_time == 0) {
	  cout << "WARNING: Computation time took longer than tick duration. Computation time: " << time_taken << " ms." << endl;
	}

	this_thread::sleep_for(chrono::milliseconds(sleep_time));

	double deltaSec =
	  std::chrono::duration<double>(hrclock::now() - startTime).count();
	/*
	  double deltaLoc =
	  sqrt(pow(posX-waypointY, 2), pow(posY-waypointY, 2));
	    
	  if(deltaLoc <=) {
	      
	  }
	*/
	    
	if(deltaSec > estimatedTimeToWpt) {
	  break;
	}
      } while (true);
    }
    return true;
  }
}
