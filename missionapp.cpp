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

#include <getopt.h>
#include <cstdlib>

#include <iostream>
#include <dronecode_sdk/dronecode_sdk.h>
#include <dronecode_sdk/action.h>
#include <dronecode_sdk/telemetry.h>
#include <signal.h>

#include <memory>
#include <chrono>
#include <thread>

#include "EnemyDrone.h"
#include "DroneUtil.h"
#include "follower_local.h"

#include "SimpleCoordinator.h"
#include "WeightedCoordinator.h"
#include "IntersectingCoordinator.h"
#include "PriorityCoordinator.h"
#include "ConjunctionCoordinator.h"
#include "RobustnessCoordinator.h"

#include "Enforcer.h"
#include "ElasticEnforcer.h"
#include "ElasticStlEnforcer.h"
#include "BoundaryEnforcer.h"
#include "RunawayEnforcer.h"
#include "FlightEnforcer.h"
#include "MissileEnforcer.h"
#include "ReconEnforcer.h"

#include "flyeightmission.h"
#include "reconmission.h"

using namespace dronecode_sdk;
using namespace std;
using namespace cdra;


// For signal handler
Mission* g_mission;
string g_outdir = ".";
EnemyDrone* g_enemy = 0;

enum ARGS {
#if	USE_ZSRM
	USE_HT,
#endif
	COORDINATOR,
	MULTIPROCESS,
	MISSION,
	OUTDIR,
	INDIR
};

static struct option long_options[] = {
	{ "coordinator",      required_argument, 0, COORDINATOR      },
	{ "mission" ,         optional_argument, 0, MISSION          },
	{ "outdir",           optional_argument, 0, OUTDIR           },
	//	{ "indir",            optional_argument, 0, INDIR            }, NOTE: BEFORE USING THIS, make the follower also use indir to find drone.cfg file!
#if USE_ZSRM
	{ "ht",               no_argument,       0, USE_HT           },
#endif
	{ "multiprocess",     optional_argument, 0, MULTIPROCESS     },
	{0, 0, 0, 0 }
};

void usage(const char* appname) {
	cout << "usage: " << appname << " [options] connection_url other_connection_url" << endl;
	cout << "Where connection_url has one of the following formats:" << endl;
	cout << "\tUDP - udp://[Bind_host][:Bind_port]" << endl;
	cout << "\tTCP - tcp://[Remote_host][:Remote_port]" << endl;
	cout << "\tSerial - serial://Dev_Node[:Baudrate]" << endl;

	cout << "valid options are:" << endl;
	int opt = 0;
	while (long_options[opt].name != 0) {
		cout << "\t--" << long_options[opt].name;
		if (long_options[opt].has_arg == required_argument) {
			cout << "=value";
		}
		cout << endl;
		opt++;
	}

	exit(EXIT_FAILURE);
}

void sig_handler(int sig) {
  if(sig == SIGINT) {
    g_enemy->kill();
    g_mission->log(g_outdir);
    fprintf(stderr, "Caught SIGINT, logging mission info.\n");
    exit(1);
  }
}

std::shared_ptr<cdra::Enforcer> make_enforcer(string enforcer_name, std::shared_ptr<dronecode_sdk::Offboard> offboard,
            std::shared_ptr<dronecode_sdk::Telemetry> telemetry, std::shared_ptr<StateStore> store) {
	std::shared_ptr<cdra::Enforcer> enforcer;

	if (enforcer_name.empty()) {
		enforcer = std::make_shared<cdra::Enforcer>(offboard, telemetry, store);
	} else if (enforcer_name == "ElasticEnforcer") {
		enforcer = std::make_shared<cdra::ElasticEnforcer>(offboard, telemetry, store);
	} else if (enforcer_name == "ElasticStlEnforcer") {
		enforcer = std::make_shared<cdra::ElasticStlEnforcer>(offboard, telemetry, store);
	} else if (enforcer_name == "BoundaryEnforcer") {
		enforcer = std::make_shared<cdra::BoundaryEnforcer>(offboard, telemetry, store);
	} else if (enforcer_name == "RunawayEnforcer") {
		enforcer = std::make_shared<cdra::RunawayEnforcer>(offboard, telemetry, store);
	} else if (enforcer_name == "FlightEnforcer") {
	        enforcer = std::make_shared<cdra::FlightEnforcer>(offboard, telemetry, store);  
	} else if (enforcer_name == "ReconEnforcer") {
	        enforcer = std::make_shared<cdra::ReconEnforcer>(offboard, telemetry, store);  
	} else if (enforcer_name == "MissileEnforcer") {
	        enforcer = std::make_shared<cdra::MissileEnforcer>(offboard, telemetry, store);  
	} else {
	  cerr << "Invalid enforcer name given." << endl;
	  exit(1);
	}

	return enforcer;
}

/* Add enforcers to coordinator */
void init_coordinator(shared_ptr<Coordinator> coordinator, map<string,float> enforcer_data, shared_ptr<Offboard> offboard, shared_ptr<Telemetry> telemetry, shared_ptr<StateStore> store) {
  for(auto kv : enforcer_data) {
    auto enforcer = make_enforcer(kv.first, offboard, telemetry, store);
    coordinator->addEnforcer(enforcer, kv.second);
  }
}

/* Add all properties being coordinated to the store */
void init_store(shared_ptr<StateStore> store, shared_ptr<Coordinator> coordinator) {
  for(auto enforcer : coordinator->getEnforcers()) {
    store->addStlExpr(dynamic_pointer_cast<StlEnforcer>(enforcer)->getProp());
  }
}

std::shared_ptr<cdra::Mission>
make_mission(string mission_name,
	     std::shared_ptr<dronecode_sdk::Offboard> offboard,
	     std::shared_ptr<dronecode_sdk::Telemetry> telemetry,
	     std::shared_ptr<dronecode_sdk::Action> action,
	     std::shared_ptr<StateStore> store,
	     cdra::Coordinator* coordinator) {
  if(mission_name == "flyeight") {
    return make_shared<FlyEightMission>(offboard, telemetry,
					action, store, coordinator);
  } else if(mission_name == "recon") {
    return make_shared<ReconMission>(offboard, telemetry, action,
				     store, coordinator);
  } else {
    fprintf(stderr, "Invalid mission name. Given %s \n", mission_name.c_str());
    exit(1);
  }
}

std::shared_ptr<cdra::Coordinator> make_coordinator(string coordinator_name, std::shared_ptr<Offboard> offboard, std::shared_ptr<Telemetry> telemetry, std::shared_ptr<StateStore> store, map<string,float> enforcer_data) {
  std::shared_ptr<cdra::Coordinator> coordinator;
  if(coordinator_name == "SimpleCoordinator") {
    coordinator = make_shared<SimpleCoordinator>(offboard, telemetry);
  } else if(coordinator_name == "WeightedCoordinator") {
    coordinator = make_shared<WeightedCoordinator>(offboard, telemetry);
  } else if(coordinator_name == "IntersectingCoordinator") {
    coordinator = make_shared<IntersectingCoordinator>(offboard, telemetry);
  } else if(coordinator_name == "PriorityCoordinator") {
    coordinator = make_shared<PriorityCoordinator>(offboard, telemetry);
  } else if(coordinator_name == "RobustnessCoordinator") {
    coordinator = make_shared<RobustnessCoordinator>(offboard, telemetry, store);
  } else if(coordinator_name == "SynthRobustnessCoordinator") { // NOTE: overrides SYNTHESIZE_ACTIONS config param
    droneutil::SYNTHESIZE_ACTIONS = 1;
    coordinator = make_shared<RobustnessCoordinator>(offboard, telemetry, store);
  } else if(coordinator_name == "ConjunctionCoordinator") {
    coordinator = make_shared<ConjunctionCoordinator>(offboard, telemetry, store);
  } else {
    fprintf(stderr, "Invalid coordinator name. Given %s \n", coordinator_name.c_str());
    exit(1);
  }
  init_coordinator(coordinator, enforcer_data, offboard, telemetry, store);
  init_store(store, coordinator);
  return coordinator;
}

bool use_ht = false;

int main(int argc, char **argv)
{ 
  string coordinator_name;      // empty string is the null enforcer
  string mission_name = "flyeight";
  string out_dir = ".";
  string in_dir  = ".";
    
  bool multiprocess = false; // Multiprocess is false by default
  
  while (1) {
    int option_index = 0;

    auto c = getopt_long(argc, argv, "", long_options, &option_index);

    if (c == -1) {
      break;
    }

    switch (c) {
    case COORDINATOR:
      coordinator_name = optarg;
      break;
    case MISSION:
      mission_name = optarg;
      break;
#if USE_ZSRM
    case USE_HT:
      use_ht = true;
      break;
#endif
    case OUTDIR:
      out_dir = optarg;
      break;
    case INDIR:
      in_dir = optarg;
      break;
    case MULTIPROCESS:
      multiprocess = true;
      break;
    default:
      usage(argv[0]);
    }
  }

  if (optind != argc - 2) { // exactly 2 non-option arg
    usage(argv[0]);
  }

  const char* url = argv[optind];
  const char* enemy_url = argv[optind+1];
	
  DronecodeSDK dc;
  ConnectionResult connection_result = dc.add_any_connection(url);
  ConnectionResult cr_enemy = dc.add_any_connection(enemy_url);

  if (connection_result != ConnectionResult::SUCCESS) {
    cout << "Connection error: " << connection_result_str(connection_result)
	 << endl;
    return EXIT_FAILURE;
  }
	
  if (cr_enemy != ConnectionResult::SUCCESS) {
    cout << "Connection error: " << connection_result_str(cr_enemy)
	 << endl;
    return EXIT_FAILURE;
  }

  // Wait for the system to connect via heartbeat
  while (dc.system_uuids().size() < 2) {
    cout << "Waiting for drones to connect" << dc.system_uuids().size() <<  endl;
    this_thread::sleep_for(chrono::seconds(1));
  }

  for (auto u : dc.system_uuids()) {
    System &s = dc.system(u);
    cout << "UUID = " << u << " SYSID = " << s.get_uuid() << endl;
  }    

  droneutil::parseConfig(in_dir+"/drone.cfg");
  
  std::thread follower_drone;
  if(!multiprocess) {
    // Run follower drone thread
    follower_drone = std::thread{ run_follower, &dc };
  } else {
    std::cout << "Using multiprocess version" << std::endl;
  }
    
  // System got discovered.
  System &dc_system = dc.system(5283920058631409232);
  auto action    = std::make_shared<Action>   (dc_system);
  auto offboard  = std::make_shared<Offboard> (dc_system);
  auto telemetry = std::make_shared<Telemetry>(dc_system);
    
  // Make an enemy drone for simulation
  System &enemy_system = dc.system(5283920058631409231);
  auto enemyDrone = std::make_shared<EnemyDrone>(enemy_system, telemetry);
  auto store = std::make_shared<StateStore>(telemetry, enemyDrone);

  std::shared_ptr<Coordinator> coordinator;

  map<string, float> enforcer_data {
    {"BoundaryEnforcer", droneutil::BOUNDARY_WEIGHT},
    {"RunawayEnforcer",  droneutil::RUNAWAY_WEIGHT},
    {"FlightEnforcer",   droneutil::FLIGHT_WEIGHT},
    //{"ReconEnforcer",  droneutil::RECON_WEIGHT},
    {"MissileEnforcer",  droneutil::MISSILE_WEIGHT}
  };

  coordinator = make_coordinator(coordinator_name, offboard,
				 telemetry, store, enforcer_data);
  
  // NOTE: passing raw coordinator pointer becauseeeee otherwise
  // I would have to change missions to take shared_ptrs 
  std::shared_ptr<Mission> mission = make_mission(mission_name, offboard, telemetry, action, store, coordinator.get());

  // This is just to write a log if we SIGINT halfway through
  g_mission = mission.get();
  g_enemy = enemyDrone.get();
  g_outdir  = out_dir;
  struct sigaction sa = {};
  sa.sa_handler = sig_handler;
  sigaction(SIGINT, &sa, NULL);

  this_thread::sleep_for(chrono::seconds(5));
  
  if(!mission->setup()) {
    fprintf(stderr, "Setup failed\n");
    action->land();
    if(!multiprocess) {
      enemyDrone->kill();
      if(follower_drone.joinable()) {
	follower_drone.join();
      }
    }
    exit(1);
  }
  
  mission->run();
  enemyDrone->kill();
  mission->cleanup();

  // Clean up if enemy drone is in same process
  if(!multiprocess) {
    if(follower_drone.joinable()) {
      follower_drone.join();
    }
  }

  mission->log(out_dir);

  return EXIT_SUCCESS;
}

