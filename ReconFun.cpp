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

#include "Signal.h"
#include "ReconFun.h"
#include "DroneUtil.h"
#include <cmath>
#include <iostream>

namespace cdra {

  ReconFun::ReconFun(float goal_z, float acceptable_range, float lowerx,
		     float lowery, float upperx, float uppery) :
    goal_z(goal_z), acceptable_range(acceptable_range),
    lowerx(lowerx), lowery(lowery), upperx(upperx), uppery(uppery) {
      
      /* What is the lowest possible robustness value that we care about */
      minValue = -acceptable_range;

      /* What is the highest robustness value that we care about? */
      /* i.e., robustness higher than this value doesn't matter */
      maxValue = computeDTE(goal_z);

      std::cout << "EMax: " << std::to_string(maxValue) << std::endl;
      std::cout << "EMin: " << std::to_string(minValue) << std::endl;
      std::cout << "EMid: " << std::to_string(-minValue / (maxValue - minValue))
		<< std::endl;
    }

    ReconFun::~ReconFun() {
    }

  bool ReconFun::isInReconZone(float ego_x, float ego_y) {
    bool in_x = ego_x >= lowerx && ego_x <= upperx;
    bool in_y = ego_y >= lowery && ego_y <= uppery;

    return in_x && in_y;
  }
  
  float ReconFun::value(Signal *sig, int t) {
    float pos_north_m = sig->value("pos_north_m", t);
    float pos_east_m  = sig->value("pos_east_m" , t);
    float pos_down_m  = sig->value("pos_down_m" , t);
    
    if(isInReconZone(pos_north_m, pos_east_m)) {
      // Pass in vertical position (note: down = -z)      
      return normalizeValue(computeDTE(-pos_down_m));
    } else {
      // If we're out of recon zone, give 0 value
      // NOTE/BEWARE: Weird interaction where you can improve robustness by avoiding recon zone
      // Makes sense in the missile case, but not the recon case
      return 0;
    }
  }
  
  float ReconFun::value(Signal *sig) {
    return value(sig, sig->length() - 1);
  }

  float ReconFun::computeDTE(float ego_z) {
    const float delta = fabsf(ego_z - goal_z);
    
    // If we're within the acceptable range, positive value
    return acceptable_range - delta; 
  }
       
  bool ReconFun::prop(Signal *sig, int t) {
    float val = value(sig, t);
    return (val >= 0);
  }

  bool ReconFun::prop(Signal *sig) {
    float val = value(sig);
    return (val >= 0);
  }
}
