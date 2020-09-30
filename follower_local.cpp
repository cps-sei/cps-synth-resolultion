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

#include <cstdlib>

#include <iostream>
#include <dronecode_sdk/dronecode_sdk.h>
#include <dronecode_sdk/action.h>
#include <dronecode_sdk/telemetry.h>
#include <dronecode_sdk/offboard.h>

#include <memory>
#include <chrono>
#include <thread>
#include <cmath>

#include "follower_local.h"
#include "DroneUtil.h"

using namespace dronecode_sdk;
using namespace std;

void run_follower(DronecodeSDK* dc)
{
  // Redundant if in same process, but that's okay
  // NOTE: assumes we're running the follower from the dir that has the drone.cfg of interest
  droneutil::parseConfig("drone.cfg");

  cout << droneutil::ENEMY_DRONE_SPEED << droneutil::CATCH_DISTANCE << endl;
  
  // System got discovered.
  System &system = dc->system(5283920058631409231);
  System &other  = dc->system(5283920058631409232);
  
  auto action    = std::make_shared<Action>(system);
  auto offboard  = std::make_shared<Offboard>(system);
  auto telemetry = std::make_shared<Telemetry>(system);

  auto otherTelemetry = std::make_shared<Telemetry>(other);
  auto otherOffboard  = std::make_shared<Offboard>(other);
  
  ActionResult action_result;
  Offboard::Result offboard_result;

  unsigned int recovery_ticks = 0;
  unsigned int TICKS_TO_RECOVER = 10;
  
  do {
    while (!telemetry->health_all_ok()) {
      cout << "Waiting for drone to be ready to arm" << endl;
      this_thread::sleep_for(chrono::seconds(1));
    }
    cout << "Drone is connected and ready" << endl;

    action_result = action->arm();
    if (action_result != ActionResult::SUCCESS) {
      cout << "Error arming drone: " << action_result_str(action_result) << endl;
      action->disarm();
      this_thread::sleep_for(chrono::seconds(3));
      continue;
    }
    cout << "Armed" << endl;
      
    action_result = action->takeoff();
    if (action_result != ActionResult::SUCCESS) {
      cout << "Error taking off: " << action_result_str(action_result) << endl;
    } else {
      this_thread::sleep_for(chrono::seconds(5));
		
      // set zero speed before starting offboard
      offboard->set_velocity_ned({0.0f, 0.0f, 0.0f, 0.0f});
      
      offboard_result = offboard->start();
	
      if (offboard_result != Offboard::Result::SUCCESS) {
	cout << "Error starting offboard mode: " << Offboard::result_str(offboard_result) << endl;
	action->land();
	this_thread::sleep_for(chrono::seconds(1));
	action->disarm();
	return;
      }
	
      // wait until the drone has ack'd it's in offboard mode
    }
  } while(offboard_result != Offboard::Result::SUCCESS || action_result != ActionResult::SUCCESS);

  cout << "Offboard mode started" << endl;

  while(!otherTelemetry->in_air()) {
    offboard->set_velocity_ned({0.0f, 0.0f, 0.0f, 0.0f});
    this_thread::sleep_for(chrono::milliseconds(10));
  }
  
  // Main loop -- Go until the ego drone is done being controlled
  do {
    Telemetry::PositionVelocityNED posvel = telemetry->position_velocity_ned();
    Telemetry::PositionNED otherPos = droneutil::pos2ned(otherTelemetry->position(), telemetry->home_position());

    // Fix z pos
    otherPos.down_m = otherTelemetry->position_velocity_ned().position.down_m;
    
    double deltaX = otherPos.north_m - posvel.position.north_m;
    double deltaY = otherPos.east_m  - posvel.position.east_m;
    
    // Only take into account up/down if we should
    float  deltaZ = droneutil::FOLLOWER_Z_VELOCITY ? otherPos.down_m  - posvel.position.down_m : 0;
			
    {
      auto &p = posvel.position;
      cout << "this @ " << p.north_m << "," << p.east_m << ", " << p.down_m << endl;
    }
    {
      auto &p = otherPos;
      cout << "other @ " << p.north_m << "," << p.east_m << ", " << p.down_m << endl;
    }

    // compute velocity vector components
    double diag = sqrt(pow(deltaX, 2) +
		       pow(deltaY, 2) + pow(deltaZ, 2));

    double velX = droneutil::ENEMY_DRONE_SPEED * deltaX / diag;
    double velY = droneutil::ENEMY_DRONE_SPEED * deltaY / diag;
    double velZ = droneutil::ENEMY_DRONE_SPEED * deltaZ / diag;
			
    double yaw = droneutil::getAngle(deltaX, deltaY);

    Offboard::VelocityNEDYaw final_vec {
      static_cast<float>(velX),
      static_cast<float>(velY),
      static_cast<float>(velZ),
      static_cast<float>(yaw)
    };

    // If we just caught enemy drone or we caught enemy drone within the last TICKS_TO_RECOVER, hold drone pos
    if (diag < droneutil::CATCH_DISTANCE || recovery_ticks > 0) {
      cout << "Enemy drone caught ego drone. " << endl;
      final_vec = { 0, 0, 0, 0 };
      if(recovery_ticks == 0) {
	recovery_ticks = TICKS_TO_RECOVER;
      } else {
	recovery_ticks--;
      }
    }

    cout << "Enemy drone velocity magnitude: " << droneutil::getMagnitude(final_vec) << endl;
    offboard->set_velocity_ned(final_vec);
			
    this_thread::sleep_for(chrono::milliseconds(std::lround(droneutil::TICK_DURATION*1000)));
  } while(otherTelemetry->in_air());

  offboard_result = offboard->stop();
  if (offboard_result != Offboard::Result::SUCCESS) {
    cout << "Error stopping offboard mode: " << Offboard::result_str(offboard_result) << endl;
  } else {
    cout << "Offboard mode stopped" << endl;
  }
  
  action_result = action->land();
  if (action_result != ActionResult::SUCCESS) {
    cout << "Error landing: " << action_result_str(action_result) << endl;
  }

  // wait until it lands
  while (telemetry->in_air()) {
    this_thread::sleep_for(chrono::seconds(1));
  }
  std::cout << "Landed" << std::endl;

  action->disarm();
}

