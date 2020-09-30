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

#ifndef MISSIONAPP_UTIL_H
#define MISSIONAPP_UTIL_H

#include <dronecode_sdk/offboard.h>
#include <dronecode_sdk/telemetry.h>
#include <stdbool.h>
#include <cmath>

namespace droneutil {
  // See explanations in DroneUtil.cpp
  extern float MAX_DRONE_SPEED;
  extern float ENEMY_CHASE_DISTANCE;
  extern float ENEMY_DRONE_SPEED;
  extern float TICK_DURATION;
  extern float TICKS_TO_CORRECT;
  
  extern bool USE_Z_VELOCITY; 
  extern bool FOLLOWER_Z_VELOCITY;
  extern bool EGO_Z_VELOCITY;

  extern float CATCH_DISTANCE;
  extern float RECON_HEIGHT;
  
  extern float BOUNDARY_WEIGHT;
  extern float RUNAWAY_WEIGHT;
  extern float FLIGHT_WEIGHT;
  extern float MISSILE_WEIGHT;
  extern float RECON_WEIGHT;

  extern bool NONLINEAR_PENALTY;
  extern bool SYNTHESIZE_ACTIONS;
  extern bool CHOOSE_LEAST_DIFFERENT_ACTION;
  extern unsigned int RANDOM_SEARCH_GRANULARITY;
  
  extern bool SUGGEST_ACTION_RANGE;

  extern unsigned int WAYPOINT_SEED;

  extern float BOUNDARY_X_MIN;
  extern float BOUNDARY_X_MAX;
  extern float BOUNDARY_Y_MIN;
  extern float BOUNDARY_Y_MAX;
  extern float BOUNDARY_Z_MIN;
  extern float BOUNDARY_Z_MAX;

  extern float BOUNDARY_SAFE_TTI_THRESHOLD;
  
  /*
  struct DroneConfig {
    float MAX_DRONE_SPEED = 2.00;       // m/s
    float ENEMY_CHASE_DISTANCE = 4.00;    // m
    float ENEMY_DRONE_SPEED = 1.6;        // m/s
    float TICK_DURATION = 0.06;           // sec
    float TICKS_TO_CORRECT = 5;
    
    bool USE_Z_VELOCITY = true;
    bool FOLLOWER_Z_VELOCITY = USE_Z_VELOCITY;
    bool EGO_Z_VELOCITY = USE_Z_VELOCITY;
    
    // When is the drone considered 'caught' 
    float CATCH_DISTANCE = 0.1;          // m
  };
  */
  
    /**
     * Ordering function; needed for set container
    * @returns true if v1.(north_m_s | east_m_s | down_m_s | yaw_deg) < v2.(...) in this order
    */
    /*
    bool lt(dronecode_sdk::Offboard::VelocityNEDYaw& ned1, dronecode_sdk::Offboard::VelocityNEDYaw& ned2) {
        return (ned1.north_m_s <  ned2.north_m_s || ned1.east_m_s < ned2.east_m_s ||
                ned1.down_m_s < ned2.down_m_s || ned1.yaw_deg < ned2.yaw_deg);
    }
     */
  
    dronecode_sdk::Offboard::VelocityNEDYaw computeNEDtoTarget
      (float curr_north_m, float curr_east_m, float curr_down_m,
       float target_north_m, float target_east_m,
       float target_down_m, float speed, float yaw);

    void parseConfig(std::string fname);

    double getAngle(double deltaX, double deltaY);
    dronecode_sdk::Telemetry::PositionNED pos2ned(const dronecode_sdk::Telemetry::Position& pos, const dronecode_sdk::Telemetry::Position& origin);
    
    bool cmp (const dronecode_sdk::Offboard::VelocityNEDYaw& v1, const dronecode_sdk::Offboard::VelocityNEDYaw& v2, float epsilon);

    void scaleVector(dronecode_sdk::Offboard::VelocityNEDYaw& vec, float new_magnitude);
    void scaleToUnitVector(dronecode_sdk::Offboard::VelocityNEDYaw& vec);
    void scaleToMaxVelocity(dronecode_sdk::Offboard::VelocityNEDYaw& vec);
    float getMagnitude(const dronecode_sdk::Offboard::VelocityNEDYaw& vec);
    double cosineSimilarity(const dronecode_sdk::Offboard::VelocityNEDYaw& v1, const dronecode_sdk::Offboard::VelocityNEDYaw& v2);
}

#endif //MISSIONAPP_UTIL_H
