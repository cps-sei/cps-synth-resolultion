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

#include "follower_local.h"

#include <memory>
#include <chrono>
#include <thread>
#include <cmath>

using namespace dronecode_sdk;
using namespace std;

void usage(const char* appname) {
	cout << "usage: " << appname << " connection_url" << endl;
	cout << "Where connection_url has one of the following formats:" << endl;
	cout << "\tUDP - udp://[Bind_host][:Bind_port]" << endl;
	cout << "\tTCP - tcp://[Remote_host][:Remote_port]" << endl;
	cout << "\tSerial - serial://Dev_Node[:Baudrate]" << endl;

	exit(EXIT_FAILURE);
}

void systemDiscoveredCallback(uint64_t uuid) {
	cout << "Drone " << uuid << " connected." << endl;
}

int main(int argc, char **argv)
{
  if (argc < 2) { // need at least one connection
    usage(argv[0]);
  }

  DronecodeSDK dc;
	
  for (int i = 1; i < argc; i++) {
    const char* url = argv[i];
    ConnectionResult connection_result = dc.add_any_connection(url);

    if (connection_result != ConnectionResult::SUCCESS) {
      cout << "Connection error: "
	   << connection_result_str(connection_result)
	   << url 
	   << endl;
      return EXIT_FAILURE;
    }
  }

  // Wait for the two drones to connect via heartbeat
  int i = 0;
  while (dc.system_uuids().size() < 2) {
    cout << "Waiting for drones to connect" << endl;
    if(i++ > 10) {
      return -1;
    }
    this_thread::sleep_for(chrono::seconds(1));
  }

  for (auto u : dc.system_uuids()) {
    System &s = dc.system(u);
    cout << "UUID = " << u << " SYSID = " << s.get_uuid() << endl;
  }

  run_follower(&dc);
  return EXIT_SUCCESS;
}

