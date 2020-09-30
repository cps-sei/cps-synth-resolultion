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

#include "TTIFun.h"
#include "Signal.h"
#include "DroneUtil.h"

#include <algorithm>
#include <iostream>
#include <math.h>

using namespace dronecode_sdk;

namespace cdra {

  TTIFun::TTIFun(float lowerx, float upperx, float lowery, float uppery,
		 float lowerz, float upperz, float safeThreshold) :
    safeThreshold(safeThreshold), lowerx(lowerx), upperx(upperx),
    lowery(lowery), uppery(uppery), lowerz(lowerz), upperz(upperz)
  {
    /* Drone has been going past some boundary for at least N=2 full seconds at max speed. */
    minValue = computeTTI(upperx + droneutil::MAX_DRONE_SPEED*2, 0, 0,
			  0, 0, 0) - safeThreshold;

    /* We don't need any sensitivity beyond 2x threshold */
    maxValue = 2*safeThreshold - safeThreshold;

    std::cout << "BMax: " << std::to_string(maxValue) << std::endl;
    std::cout << "BMin: " << std::to_string(minValue) << std::endl;
    std::cout << "BMid: " << std::to_string(-minValue / (maxValue - minValue)) << std::endl;
  }

  TTIFun::~TTIFun()
  {
  }
  
  float TTIFun::value(Signal *sig, int t) {
    if (sig->length() - 1 < t)
      throw "Signal unavailable!";
    float pos_east_m    = sig->value("pos_east_m"   , t);
    float pos_north_m   = sig->value("pos_north_m"  , t);
    float pos_down_m    = sig->value("pos_down_m"   , t);
    float vel_east_m_s  = sig->value("vel_east_m_s" , t);
    float vel_north_m_s = sig->value("vel_north_m_s", t);
    float vel_down_m_s  = sig->value("vel_down_m_s" , t);
    float tti = computeTTI(pos_east_m, pos_north_m, -pos_down_m,
			   vel_east_m_s, vel_north_m_s, -vel_down_m_s);
    //    std::cout << "TTI: " << tti << " Normalized value: " << normalizeValue(tti - safeThreshold) << std::endl;
    return normalizeValue(tti - safeThreshold);
  }
  
  float TTIFun::value(Signal *sig) {
    return value(sig, sig->length() - 1);
  }

  bool TTIFun::closeToXBoundary(float pos_east_m, float pos_north_m, float pos_down_m,
                                float vel_east_m_s, float vel_north_m_s, float vel_down_m_s) const {
    return
      closeToLowerX(pos_east_m, pos_north_m, pos_down_m, vel_east_m_s, vel_north_m_s, vel_down_m_s) ||
      closeToUpperX(pos_east_m, pos_north_m, pos_down_m, vel_east_m_s, vel_north_m_s, vel_down_m_s);
  }

  bool TTIFun::closeToYBoundary(float pos_east_m, float pos_north_m, float pos_down_m,
                                  float vel_east_m_s, float vel_north_m_s, float vel_down_m_s) const {
    return
      closeToLowerY(pos_east_m, pos_north_m, pos_down_m, vel_east_m_s, vel_north_m_s, vel_down_m_s) ||
      closeToUpperY(pos_east_m, pos_north_m, pos_down_m, vel_east_m_s, vel_north_m_s, vel_down_m_s);
  }

  bool TTIFun::closeToLowerX(float pos_east_m, float pos_north_m, float pos_down_m,
                             float vel_east_m_s, float vel_north_m_s, float vel_down_m_s) const {
    return pos_north_m < lowerx || (vel_north_m_s < 0 && fabsf(lowerx - pos_north_m) / (-vel_north_m_s) < safeThreshold);
  }

  bool TTIFun::closeToUpperX(float pos_east_m, float pos_north_m, float pos_down_m,
                               float vel_east_m_s, float vel_north_m_s, float vel_down_m_s) const{
    return pos_north_m > upperx || (vel_north_m_s > 0 && fabsf(upperx - pos_north_m) / (vel_north_m_s) < safeThreshold);
  }

  bool TTIFun::closeToLowerY(float pos_east_m, float pos_north_m, float pos_down_m,
                               float vel_east_m_s, float vel_north_m_s, float vel_down_m_s) const{
    return pos_east_m < lowery || (vel_east_m_s < 0 && fabsf(lowery - pos_east_m) / (-vel_east_m_s) < safeThreshold);
  }

  bool TTIFun::closeToUpperY(float pos_east_m, float pos_north_m, float pos_down_m,
                               float vel_east_m_s, float vel_north_m_s, float vel_down_m_s) const{
    return pos_east_m > uppery || (vel_east_m_s > 0 && fabsf(uppery - pos_east_m) / (vel_east_m_s) < safeThreshold);
  }
  bool TTIFun::closeToLowerZ(float pos_up_m, float vel_up_m_s) const{
    return pos_up_m < lowerz || (vel_up_m_s < 0 && fabsf(lowerz - pos_up_m) / (-vel_up_m_s) < safeThreshold);
  }

  bool TTIFun::closeToUpperZ(float pos_up_m, float vel_up_m_s) const{
    return pos_up_m > upperz || (vel_up_m_s > 0 && fabsf(upperz - pos_up_m) / (vel_up_m_s) < safeThreshold);
  }

  bool TTIFun::closeToZBoundary(float pos_up_m, float vel_up_m_s) const{
    return closeToUpperZ(pos_up_m, vel_up_m_s) || closeToLowerZ(pos_up_m, vel_up_m_s);
  }
  
  float TTIFun::computeTTI(float pos_east_m, float pos_north_m, float pos_up_m,
			   float vel_east_m_s, float vel_north_m_s, float vel_up_m_s) const
  {
    float res = 1000.0;

    /* Time to hit boundary if within boundary. 
     * If outside boundary, provides meaningful negative value */
    if(pos_north_m <= lowerx) { // Below
      if(vel_north_m_s <= 0.0f) res = std::min(res, (pos_north_m - lowerx) + (vel_north_m_s));
      if(vel_north_m_s > 0.0f) res = std::min(res, (pos_north_m - lowerx) / (vel_north_m_s));
    } else if(pos_north_m >= upperx) { // Above
      if(vel_north_m_s < 0.0f) res = std::min(res, (upperx - pos_north_m) / (vel_north_m_s));
      if(vel_north_m_s >= 0.0f) res = std::min(res, (upperx - pos_north_m) - (vel_north_m_s));
    } else { // In boundary (wrt N/S)
      if(vel_north_m_s < 0) res = std::min(res, fabsf(lowerx - pos_north_m) / (-vel_north_m_s));
      if(vel_north_m_s > 0) res = std::min(res, fabsf(upperx - pos_north_m) / (vel_north_m_s));
    }
    if(pos_east_m <= lowery) { // Below
      if(vel_east_m_s <= 0.0f) res = std::min(res, (pos_east_m - lowery) + (vel_east_m_s));
      if(vel_east_m_s > 0.0f) res = std::min(res, (pos_east_m - lowery) / (vel_east_m_s));
    } else if(pos_east_m >= uppery) { // Above
      if(vel_east_m_s < 0.0f) res = std::min(res, (uppery - pos_east_m) / (vel_east_m_s));
      if(vel_east_m_s >= 0.0f) res = std::min(res, (uppery - pos_east_m) - (vel_east_m_s));
    } else { // In boundary (wrt E/W)
      if(vel_east_m_s < 0) res = std::min(res, fabsf(lowery - pos_east_m) / (-vel_east_m_s));
      if(vel_east_m_s > 0) res = std::min(res, fabsf(uppery - pos_east_m) / (vel_east_m_s));
    }
    if(pos_up_m <= lowerz) { // Below
      if(vel_up_m_s <= 0.0f) res = std::min(res, (pos_up_m - lowerz) + (vel_up_m_s));
      if(vel_up_m_s > 0.0f) res = std::min(res, (pos_up_m - lowerz) / (vel_up_m_s));
    } else if(pos_up_m >= upperz) { // Above 
      if(vel_up_m_s < 0.0f) res = std::min(res, (upperz - pos_up_m) / (vel_up_m_s));
      if(vel_up_m_s >= 0.0f) res = std::min(res, (upperz - pos_up_m) - (vel_up_m_s));
    } else { // In boundary (wrt z-axis)
      if(vel_up_m_s < 0) res = std::min(res, fabsf(lowerz - pos_up_m) / (-vel_up_m_s));
      if(vel_up_m_s > 0) res = std::min(res, fabsf(upperz - pos_up_m) / (vel_up_m_s));
    }

    /*
    std::cout << "TTI-in: " << pos_north_m << ", " << pos_east_m << ", " << pos_down_m;
    std::cout << "[" << vel_north_m_s << ", " << vel_east_m_s << ", " << vel_down_m_s << "]" << std::endl;
    std::cout << "TTI-out: " << res << std::endl;
    */
    return res;
  }

  
  bool TTIFun::prop(Signal *sig, int t) {
    float val = value(sig, t);
    return (val >= 0);
  }
  
  bool TTIFun::prop(Signal *sig) {
    float val = value(sig);
    return (val >= 0);
  }

}
