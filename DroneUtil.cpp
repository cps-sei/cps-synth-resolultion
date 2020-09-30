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

#include "DroneUtil.h"
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>

using namespace dronecode_sdk;

namespace droneutil {

  float MAX_DRONE_SPEED = 2.00;         // m/s
  float ENEMY_CHASE_DISTANCE = 4.00;    // m
  float ENEMY_DRONE_SPEED = 1.6;        // m/s
  float TICK_DURATION = 0.06;           // sec
  float TICKS_TO_CORRECT = 5;           // Used as the length of the prediction window in estimating signal -- Could be used to make properties violated for N ticks as well

  // NOTE: turning off z-velocity stuff might be a bit broken right now -- not supported with all new enforcers
  bool USE_Z_VELOCITY = true;           // Should we use z velocity? (changes follower and ego z velocity)
  bool FOLLOWER_Z_VELOCITY = USE_Z_VELOCITY; 
  bool EGO_Z_VELOCITY = USE_Z_VELOCITY;
    
  float CATCH_DISTANCE = 0.1;          // When is the drone considered to be 'caught'
  float RECON_HEIGHT   = 1.2;          // Desired altitude of recon mission -- used by reconmission.h
  
  float BOUNDARY_WEIGHT = 2;           
  float RUNAWAY_WEIGHT  = 3;
  float FLIGHT_WEIGHT   = 10;
  float RECON_WEIGHT    = 1;           // Unused
  float MISSILE_WEIGHT  = 3;

  bool NONLINEAR_PENALTY  = true;      // Used in SigFun.cpp

  
  bool SYNTHESIZE_ACTIONS = true; // Only relevant to RobustnessCoordinator, overwritten by SynthRobustnessCoordinator (to true)
  bool CHOOSE_LEAST_DIFFERENT_ACTION = true; // Only relevant to (Synth|)RobustnessCoordinator
  unsigned int RANDOM_SEARCH_GRANULARITY = 10; // Only relevant to RobustnessCoordinator w/ synthesis -- determines how rigorously to search the action range (higher=more)
  
  bool SUGGEST_ACTION_RANGE = true; // Used by each enforcer -- if false, each only proposes a single action

  unsigned int WAYPOINT_SEED = 0; // Used by reconmission to randomly generate waypoints

  float BOUNDARY_X_MIN = -10;
  float BOUNDARY_X_MAX = 10;
  float BOUNDARY_Y_MIN = -10;
  float BOUNDARY_Y_MAX = 10;
  float BOUNDARY_Z_MIN = 0;
  float BOUNDARY_Z_MAX = 6;

  float BOUNDARY_SAFE_TTI_THRESHOLD = 1.5; // Safe TTI threshold used by BoundaryEnforcer
  
  void scaleVector(Offboard::VelocityNEDYaw& vec, float new_magnitude) {
    float cur_magnitude  = getMagnitude(vec);
    float scaling_factor = new_magnitude/cur_magnitude;
  
    vec.north_m_s *= scaling_factor;
    vec.east_m_s  *= scaling_factor;
    vec.down_m_s  *= scaling_factor;
  }

  void scaleToUnitVector(Offboard::VelocityNEDYaw& vec) {
    scaleVector(vec, 1);
  }

  void scaleToMaxVelocity(dronecode_sdk::Offboard::VelocityNEDYaw& vec) {
    scaleVector(vec, droneutil::MAX_DRONE_SPEED);
  }

  
  float getMagnitude(const Offboard::VelocityNEDYaw& vec) {
    return sqrt(vec.north_m_s*vec.north_m_s + vec.east_m_s*vec.east_m_s + vec.down_m_s*vec.down_m_s);
  }

  // Compute the velocity vector from "current" to "target"
  Offboard::VelocityNEDYaw computeNEDtoTarget(float curr_north_m, float curr_east_m, float curr_down_m,
					      float target_north_m, float target_east_m, float target_down_m,
					      float speed, float yaw){
      
    float delta_north = target_north_m - curr_north_m;
    float delta_east  = target_east_m  - curr_east_m;
    float delta_down  = EGO_Z_VELOCITY ? target_down_m - curr_down_m : 0;
    
    float delta = sqrt(pow(delta_north, 2) + pow(delta_east, 2) + pow(delta_down, 2));
    
    Offboard::VelocityNEDYaw velNED;
    velNED.north_m_s = (delta_north / delta) * speed;
    velNED.east_m_s  = (delta_east  / delta) * speed;
    velNED.down_m_s  = (delta_down  / delta) * speed;
    velNED.yaw_deg = yaw;
    
    return velNED;
  }


  /* grosssssssss */
  void setVar(std::string name, float value) {
    if(name == "MAX_DRONE_SPEED") {
      MAX_DRONE_SPEED = value;
    } else if(name == "ENEMY_CHASE_DISTANCE") {
      ENEMY_CHASE_DISTANCE = value;
    } else if(name == "ENEMY_DRONE_SPEED") {
      ENEMY_DRONE_SPEED = value;
    } else if(name == "TICK_DURATION") {
      TICK_DURATION = value;
    } else if(name == "TICKS_TO_CORRECT") {
      TICKS_TO_CORRECT = value;
    } else if(name == "USE_Z_VELOCITY") {
      USE_Z_VELOCITY = value != 0;
      FOLLOWER_Z_VELOCITY = USE_Z_VELOCITY;
      EGO_Z_VELOCITY = USE_Z_VELOCITY;
    } else if(name == "CATCH_DISTANCE") {
      CATCH_DISTANCE = value;
    } else if(name == "BOUNDARY_WEIGHT") {
      BOUNDARY_WEIGHT = value;
    } else if(name == "RUNAWAY_WEIGHT") {
      RUNAWAY_WEIGHT = value;
    } else if(name == "FLIGHT_WEIGHT") {
      FLIGHT_WEIGHT = value;
    } else if(name == "MISSILE_WEIGHT") {
      MISSILE_WEIGHT = value;
    } else if(name == "RECON_WEIGHT") {
      RECON_WEIGHT = value;
    } else if(name == "RECON_HEIGHT") {
      RECON_HEIGHT = value;
    } else if(name == "NONLINEAR_PENALTY") {
      NONLINEAR_PENALTY = value != 0;
    } else if(name == "SYNTHESIZE_ACTIONS") {
      SYNTHESIZE_ACTIONS = value != 0;
    } else if(name == "CHOOSE_LEAST_DIFFERENT_ACTION") {
      CHOOSE_LEAST_DIFFERENT_ACTION = value != 0;
    } else if(name == "SUGGEST_ACTION_RANGE") {
      SUGGEST_ACTION_RANGE = value != 0;
    } else if(name == "WAYPOINT_SEED") {
      WAYPOINT_SEED = (unsigned int)value;
    } else if(name == "BOUNDARY_X_MIN") {
      BOUNDARY_X_MIN = value;
    } else if(name == "BOUNDARY_X_MAX") {
      BOUNDARY_X_MAX = value;
    } else if(name == "BOUNDARY_Y_MIN") {
      BOUNDARY_Y_MIN = value;
    } else if(name == "BOUNDARY_Y_MAX") {
      BOUNDARY_Y_MAX = value;
    } else if(name == "BOUNDARY_Z_MIN") {
      BOUNDARY_Z_MIN = value;
    } else if(name == "BOUNDARY_SAFE_TTI_THRESHOLD") {
      BOUNDARY_SAFE_TTI_THRESHOLD = value;
    } else if(name == "BOUNDARY_SIZE") {
      BOUNDARY_X_MIN = -value;
      BOUNDARY_X_MAX = value;
      BOUNDARY_Y_MIN = -value;
      BOUNDARY_Y_MAX = value;
      BOUNDARY_Z_MIN = -value; // We don't really want this anywho
      BOUNDARY_Z_MAX = value;
    } else if(name == "RANDOM_SEARCH_GRANULARITY") {
      RANDOM_SEARCH_GRANULARITY = value;
    } else {
      fprintf(stderr, "Unknown variable name: %s, %f\n", name.c_str(), value);
    }
  }
  
  void parseConfig(std::string fname){
    std::ifstream infile(fname);
    std::string line;

    while (std::getline(infile, line)) {
      std::istringstream iss(line);
      std::string name;
      float val;
     
      if (!(iss >> name >> val)) { break; } // error
      
      setVar(name, val);      

      std::cout << name << ":" << val << std::endl;
    }
  }
  
  double getAngle(double deltaX, double deltaY){
    double deg = atan(deltaY / deltaX) * 180.0 / M_PI;
    
    // for clock-wise 0-north angles
    if (deltaX < 0) {
      deg += 180;
    } else if (deltaY < 0) {
      deg += 360;
    }
    
    return deg;
  }
  
  dronecode_sdk::Telemetry::PositionNED pos2ned(const dronecode_sdk::Telemetry::Position& pos, const dronecode_sdk::Telemetry::Position& origin) {
    dronecode_sdk::Telemetry::PositionNED ned;

    // based on formulas from https://www.movable-type.co.uk/scripts/latlong.html

    const double R = 6371e3; // metres
    const double DEG_TO_RAD = M_PI / 180.0;

    double phi1 = origin.latitude_deg * DEG_TO_RAD;
    double phi2 = pos.latitude_deg * DEG_TO_RAD;

    double deltaLambda = (pos.longitude_deg - origin.longitude_deg) * DEG_TO_RAD;

    double x = deltaLambda * cos((phi1 + phi2)/2);
    double y = (phi2-phi1);
	
    ned.north_m = y * R;
    ned.east_m  = x * R;

    /* NOTE/IMPORTANT: This is inaccurate a lot of the time bc origin altitudes vary depending on some timing stuff?
     * Need to fix z velocity outside */
    // negative bc down position is opposite up position (altitude)
    ned.down_m  = -(pos.absolute_altitude_m - origin.absolute_altitude_m);
    ned.down_m  = -(pos.absolute_altitude_m - origin.absolute_altitude_m);    
    
    return ned;
}
  
  bool cmp(const dronecode_sdk::Offboard::VelocityNEDYaw& v1, const dronecode_sdk::Offboard::VelocityNEDYaw& v2, float epsilon){
    // Ensure magnitudes are equivalent before comparing -- NOTE: scale the vectors if they're not
    assert(getMagnitude(v1) == getMagnitude(v2));
    
    bool is_eq =
      (fabsf(v1.north_m_s - v2.north_m_s) < epsilon) &&
      (fabsf(v1.east_m_s  - v2.east_m_s ) < epsilon) &&
      (fabsf(v1.down_m_s  - v2.down_m_s ) < epsilon);

    return is_eq;
  }

  /* 
   * Quantify the similarity between two vectors
   * Using cosine similarity measure
   * Note: Doesn't take into account vector magnitude
   * https://en.wikipedia.org/wiki/Cosine_similarity
   */
  double cosineSimilarity(const Offboard::VelocityNEDYaw& v1, const Offboard::VelocityNEDYaw& v2) {
    double mag_product = getMagnitude(v1) * getMagnitude(v2);
    double dot_product = v1.north_m_s * v2.north_m_s +
                         v1.east_m_s  * v2.east_m_s  +
                         v1.down_m_s  * v2.down_m_s;
    return dot_product/mag_product;
  }

}
