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
#include <cmath>

#include "SigFun.h"
#include "Signal.h"
#include "DroneUtil.h"

namespace cdra {
  
float SigFun::value(Signal *sig, int t) {
  // Default function just returns 0
  return 0;
}
  
float SigFun::value(Signal *sig) {
  // Default function just returns 0
  return 0;
}
  
float scaleToCurve(float x) {
  // Given value in [-1, 0], scales exponentially within [-2, 0]
  // Such that all negative values are higher than their corresponding pos values
  // Eq: -((a^-x - 1) / (a-1)) + x
  // Higher a values will make it 'more exponential' (less linear)
  // 
  float base = 32;
  return -((pow(base, -x) - 1) / (base - 1)) + x;
}
  
float SigFun::normalizeValue(float value) {
  if(value == 0) {
    return 0;
  } else if(value < 0) {
    // Only want sensitivity between minValue and maxValue
    // Truncate if outside range  
    value = value < minValue ? minValue : value;
    float penalty_factor = 2;
    // Normalize negative values to [-2, 0]
    // Scale to exponential curve if desired
    if(droneutil::NONLINEAR_PENALTY) {
      return (scaleToCurve(((value - minValue) / (0 - minValue) - 1)));
    } else {
      return penalty_factor*((value-minValue) / (0 - minValue) - 1);
    }
  } else {
    // Only want sensitivity between minValue and maxValue
    // Truncate if outside range  
    value = value > maxValue ? maxValue : value;

    // Normalize positive values to [0, 1]
    return ((value - 0) / (maxValue - 0));
  }
}
  
bool SigFun::prop(Signal *sig, int t) {
  // Default function just returns true
  return true;
}

bool SigFun::prop(Signal *sig) {
  
  // Default function just returns true
  return true;
}
  
}
