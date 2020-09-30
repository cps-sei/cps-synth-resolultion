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
#include "DTGFun.h"
#include "DroneUtil.h"
#include <cmath>
#include <iostream>

namespace cdra {

  DTGFun::DTGFun(float safeDist) : safeDist(safeDist) {
      
    /* What is the lowest possible robustness value */
    minValue = 0 - safeDist;
    
    /* What is the highest robustness value that we care about? */
    /* i.e., robustness higher than this value doesn't matter */
    maxValue = safeDist*2 - safeDist;

    std::cout << "GMax: " << std::to_string(maxValue) <<  std::endl;
    std::cout << "GMin: " << std::to_string(minValue) << std::endl;
    std::cout << "GMid: " << std::to_string(-minValue / (maxValue - minValue))
	      << std::endl;
  }

  DTGFun::~DTGFun() {
  }
  
  float DTGFun::value(Signal *sig, int t) {
    float pos_down_m = sig->value("pos_down_m", t);
    
    // Pass in vertical position (note: down = -z)
    auto dtg = computeDTG(-pos_down_m);
	
    return normalizeValue(dtg - safeDist);
  }
  
  float DTGFun::value(Signal *sig) {
    return value(sig, sig->length() - 1);
  }

  float DTGFun::computeDTG(float ego_z) {
    const float delta = ego_z - ground_z;
    return delta;
  }
  
  bool DTGFun::prop(Signal *sig, int t) {
    float val = value(sig, t);
    return (val >= 0);
  }

  bool DTGFun::prop(Signal *sig) {
    float val = value(sig);
    return (val >= 0);
  }
}
