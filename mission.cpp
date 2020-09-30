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

#include <thread>
#include <chrono>
#include <iostream>

#include <dronecode_sdk/dronecode_sdk.h>
#include <dronecode_sdk/offboard.h>
#include <dronecode_sdk/telemetry.h>
#include <dronecode_sdk/action.h>

#include "mission.h"
#include "StateStore.h"
#include "Coordinator.h"

using namespace std;
using namespace dronecode_sdk;

namespace cdra {
  
  Mission::Mission(std::shared_ptr<dronecode_sdk::Offboard> offboard,
		   std::shared_ptr<dronecode_sdk::Telemetry> telemetry,
		   std::shared_ptr<dronecode_sdk::Action> action,
		   std::shared_ptr<cdra::StateStore> store,
		   cdra::Coordinator* coordinator)
    : offboard(offboard), telemetry(telemetry), action(action), store(store), coordinator(coordinator){ }
  
  bool Mission::setup() {
    // Wait for drone to be ready to arm
    while (!telemetry->health_all_ok()) {
      cout << "Waiting for drone to be ready to arm" << endl;
      cout << telemetry->health() << endl;
      this_thread::sleep_for(chrono::seconds(1));
    }
    cout << "Drone is connected and ready" << endl;

    // Try to arm drone
    ActionResult action_result = action->arm();
    {
      int i = 0;
      while (action_result != ActionResult::SUCCESS && i++ < 10) {
	cout << "Error arming drone: " << action_result_str(action_result) << endl;
	action->disarm();
	this_thread::sleep_for(chrono::seconds(5));
      }
      if(action_result != ActionResult::SUCCESS) { return false; }
    }
    
    cout << "Armed" << endl;

    // Takeoff
    action_result = action->takeoff();
    if (action_result != ActionResult::SUCCESS) {
      cout << "Error taking off: " << action_result_str(action_result) << endl;
      return false;
    }

    offboard->set_velocity_ned({0.0f, 0.0f, 0.0f, 0.0f});
    
    while(!telemetry->in_air()) {
      offboard->set_velocity_ned({0.0f, 0.0f, 0.0f, 0.0f});
      this_thread::sleep_for(chrono::milliseconds(10));
    }
    
    Offboard::Result offboard_result = offboard->start();
    if(offboard_result != Offboard::Result::SUCCESS) {
      cout << "Error starting offboard mode: " << Offboard::result_str(offboard_result) << endl;
      return false;
    }
  
    return true;
  }

  bool Mission::log(string logdir) {

    // Store the entire signal
    store->writeSignal({"enemy_pos_east_m", "enemy_pos_north_m"}, logdir+"/plot_enemy_drone.dat");
    store->writeSignal({
	"pos_east_m", "pos_north_m", "pos_down_m",
	"vel_east_m_s", "vel_north_m_s", "vel_down_m_s"
	"enemy_pos_east_m", "enemy_pos_north_m", "enemy_pos_down_m",
	"enemy_vel_east_m_s", "enemy_vel_north_m_s", "enemy_vel_down_m_s"},
      logdir + "/statestore.log");
    
    // write drone locations over time to file to be plotted
    store->writeSignal({"pos_east_m", "pos_north_m"}, logdir+"/plot_my_drone.dat");

    // Write stuff for animation
    store->writeAnimationData(logdir+"/");

    // Write interesting points
    store->writeChasePoints(logdir+"/plot_chase_points.dat");
    store->writeCoordinatedPoints(logdir+"/plot_coordinated_points.dat");
    store->writeCoordinatorActivity(logdir+"/");
    store->writeJSONData(logdir+"/data.json");
    std::cout << "Wrote data files." << std::endl;

    return true;
  }
  
  bool Mission::cleanup() {
    ActionResult action_result;
    
    // Begin landing
    action_result = action->land();
    if (action_result != ActionResult::SUCCESS) {
      cout << "Error landing: " << action_result_str(action_result)
	   << endl;
    }
    
    // wait until it touches ground
    while (telemetry->in_air()) {
      this_thread::sleep_for(chrono::seconds(1));
    }
    std::cout << "Landed" << std::endl;

    // Note: not checking success (because it doesn't really matter at this point)
    action->disarm();

    return true;
  }
  
}
