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
#include "DTTFun.h"
#include "DroneUtil.h"
#include <cmath>
#include <iostream>

namespace cdra {

    DTTFun::DTTFun(float safeDist) : safeDist(safeDist)
    {
      /* What is the lowest possible robustness value (0 is actual lowest, but drone is considered 'caught' if it's within */
      /* Realistically want lower than CATCH_DISTANCE+N to be the min */
      minValue = droneutil::CATCH_DISTANCE+.1 - safeDist;

      /* What is the highest robustness value that we care about? */
      /* i.e., robustness higher than this value doesn't matter */
      maxValue = safeDist*2 - safeDist;

      std::cout << "RMax: " << std::to_string(maxValue) <<  std::endl;
      std::cout << "RMin: " << std::to_string(minValue) << std::endl;
      std::cout << "RMid: " << std::to_string(-minValue / (maxValue - minValue)) << std::endl;
    }

    DTTFun::~DTTFun()
    {
    }

    float DTTFun::value(Signal *sig, int t) {
        float pos_east_m  = sig->value("pos_east_m" , t);
        float pos_north_m = sig->value("pos_north_m", t);
        float pos_down_m  = sig->value("pos_down_m" , t);
        float enemy_pos_east_m  = sig->value("enemy_pos_east_m" , t);
        float enemy_pos_north_m = sig->value("enemy_pos_north_m", t);
        float enemy_pos_down_m  = sig->value("enemy_pos_down_m" , t);
        float dtt = computeDTT(pos_east_m, pos_north_m, pos_down_m,
                               enemy_pos_east_m, enemy_pos_north_m, enemy_pos_down_m);
        return normalizeValue(dtt - safeDist);
    }
  
    float DTTFun::value(Signal *sig) {
        return value(sig, sig->length() - 1);
    }

    float DTTFun::computeDTT(float x1, float y1, float z1, float x2, float y2, float z2) const
    {
        const float delta = sqrt(pow(x2-x1, 2.0) + pow(y2-y1, 2.0) + pow(z2-z1, 2.0));
        return delta;
    }
  
    bool DTTFun::prop(Signal *sig, int t) {
        float val = value(sig, t);
        return (val >= 0);
    }

    bool DTTFun::prop(Signal *sig) { 
        float val = value(sig);
        return (val >= 0);
    }

}
