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

#include <iostream>
#include <chrono>
#include <stdbool.h>
#include <iostream>
#include <memory>
#include <cmath>
#include <thread>

#include "mission.h"
#include "reconmission.h"
#include "DroneUtil.h"

using namespace std;
using namespace dronecode_sdk;
using hrclock = std::chrono::high_resolution_clock;

namespace cdra {
ReconMission::ReconMission(std::shared_ptr<dronecode_sdk::Offboard> offboard,
			   std::shared_ptr<dronecode_sdk::Telemetry> telemetry,
			   std::shared_ptr<dronecode_sdk::Action> action,
			   std::shared_ptr<cdra::StateStore> store,
			   cdra::Coordinator* coordinator)
  : Mission(offboard, telemetry, action, store, coordinator){ }

  Telemetry::PositionNED get_position(unsigned int quadrant, float sidelen) {
    
    float x = static_cast<float>(rand() % static_cast<unsigned int>(sidelen));
    float y = static_cast<float>(rand() % static_cast<unsigned int>(sidelen));
    
    if(quadrant == 1) {
      // keep x and y positive
    } else if(quadrant == 2) {
      x = -x;
      // keep y positive
    } else if(quadrant == 3) {
      x = -x;
      y = -y;
    } else if(quadrant == 4) {
      // keep x positive
      y = -y;
    } else {
      fprintf(stderr, "Invalid quadrant given to get_position. Must be in {1,2,3,4}. \n");
      exit(1);
    }
    return {x, y, 0.0f};
}
  
bool ReconMission::run() {
  // NOTE: usable distance is how far the waypoint can be from the boundary without boundaryenforcer interfering too bad
  // NOTE: assuming we are only using square xy boundary centered at 0,0 (i.e., |xmax| = |xmin| = |ymax| = |ymin|)
  float usable_sidelen = droneutil::BOUNDARY_X_MAX - (droneutil::MAX_DRONE_SPEED / droneutil::BOUNDARY_SAFE_TTI_THRESHOLD);

  vector<Telemetry::PositionNED> waypoints;
  // 0 seed just do a box
  if(droneutil::WAYPOINT_SEED == 0) {
    waypoints = {
      {usable_sidelen , usable_sidelen , 0}, // Top right
      {usable_sidelen , -usable_sidelen, 0}, // Top left
      {-usable_sidelen, -usable_sidelen, 0}, // Bottom left
      {-usable_sidelen, usable_sidelen , 0}, // Bottom right
    };
  } else {
    srand(droneutil::WAYPOINT_SEED);
    waypoints = {
      get_position(1, usable_sidelen),
      get_position(2, usable_sidelen),
      get_position(3, usable_sidelen),
      get_position(4, usable_sidelen)
    };

    // Shuffle order of waypoints
    std::random_shuffle(waypoints.begin(), waypoints.end());
  }

  cout << "Waypoints: ";
  for(auto waypoint : waypoints) {
    cout << "[" << waypoint.north_m <<  ", " << waypoint.east_m << "] ";
  }
  cout << endl;
  for(auto waypoint : waypoints) {
    fly_to_waypoint(waypoint);
    perform_recon();
    cout << "flew to waypoint" << endl;
  }

  // Go back to origin-ish
  fly_to_waypoint({0,0,0});
  cout << "flew to waypoint (origin, we're done) " << endl;
  
  return true;
}

void ReconMission::fly_to_waypoint(Telemetry::PositionNED waypoint) {
  float goal_x = waypoint.north_m;
  float goal_y = waypoint.east_m;
  
  float cur_x, cur_y, diag;
  Telemetry::PositionVelocityNED posvel;
  Telemetry::PositionNED pos;
  
  do {

    // Record the latest state info
    store->recordNewState();
    
    // Determine current location
    posvel = telemetry->position_velocity_ned();
    pos = posvel.position;
    cur_x = pos.north_m;
    cur_y = pos.east_m;
    
    // Determine velocity vector to get to next location
    float deltaX = goal_x - cur_x;
    float deltaY = goal_y - cur_y;
    
    diag = sqrt(pow(deltaX, 2) + pow(deltaY, 2));

    float velX = droneutil::MAX_DRONE_SPEED * deltaX / diag;
    float velY = droneutil::MAX_DRONE_SPEED * deltaY / diag;
    float yaw  = droneutil::getAngle(deltaX, deltaY);
    
    auto start_time = chrono::high_resolution_clock::now();
    coordinator->sendVelocityNed({velX, velY, 0.0f, 0.0f});	    
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

  } while(diag > 0.5);
}

void ReconMission::perform_recon() {

  Telemetry::PositionVelocityNED posvel = telemetry->position_velocity_ned();
  auto pos = posvel.position;

  auto original_position = pos;
  
  const float angularStep = -0.1;
  const float figureLength = 1;
  float theta = 2.5 * M_PI;

  double waypointX = pos.north_m;
  double waypointY = pos.east_m;
  const double waypointZ = droneutil::RECON_HEIGHT;
  bool firstWpt = true;

  while (theta > 0.5 * M_PI) {
    posvel = telemetry->position_velocity_ned();
    pos = posvel.position;
    
    float prevWaypointX = waypointX;
    float prevWaypointY = waypointY;
    double scale = 2 / (3 - cos(2 * theta));
    
    waypointX = figureLength * scale * cos(theta) + original_position.north_m;
    waypointY = figureLength * scale * sin(2 * theta) / 2 + original_position.east_m;

    theta += angularStep;

    if (firstWpt) {
      firstWpt = false;
      continue;
    }

    // Note: Z waypoint is fixed
    double distance = sqrt(pow(waypointX - prevWaypointX, 2) + pow(waypointY - prevWaypointY, 2));
    double estimatedTimeToWpt = distance / droneutil::MAX_DRONE_SPEED;
    
    auto startTime = hrclock::now();

    // Main fly-8 loop
    do {
      // is that point behind?

      // Record the latest state info. in the global state storage
      store->recordNewState();
	    
      posvel = telemetry->position_velocity_ned();
      float posX = posvel.position.north_m;
      float posY = posvel.position.east_m;
      float posZ = -posvel.position.down_m; // Want 'up' position, not down position
      
      float yaw = telemetry->attitude_euler_angle().yaw_deg; // 0-north clockwise

      double deltaX = waypointX - posX;
      double deltaY = waypointY - posY;
      double deltaZ = waypointZ - posZ;

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
      double diag = sqrt(pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2));
      double velX = droneutil::MAX_DRONE_SPEED * deltaX / diag;
      double velY = droneutil::MAX_DRONE_SPEED * deltaY / diag;
      double velZ = droneutil::MAX_DRONE_SPEED * deltaZ / diag;
      
      double figureYaw = droneutil::getAngle(waypointX - prevWaypointX, waypointY - prevWaypointY);
	    
      auto start_time = chrono::high_resolution_clock::now();
      coordinator->sendVelocityNed({(float) velX, (float) velY, (float) -velZ, (float) 0.0f});	    
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

      if(deltaSec > estimatedTimeToWpt) {
	break;
      }
    } while (true);
  }
  
  // Go back up to usual height
    while(-telemetry->position_velocity_ned().position.down_m < 2.0f) {
      store->recordNewState();
      auto start_time = chrono::high_resolution_clock::now();
      coordinator->sendVelocityNed({0.0f, 0.0f, -2.0f, 0.0f});
      auto end_time = chrono::high_resolution_clock::now();
      auto time_taken = chrono::duration_cast<chrono::milliseconds>(end_time-start_time).count();
      auto sleep_time = max((int)(std::lround(droneutil::TICK_DURATION*1000)-time_taken), 0);
      
      this_thread::sleep_for(chrono::milliseconds(sleep_time));
    }
}
}

