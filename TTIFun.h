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

#ifndef TTIFUN_H_
#define TTIFUN_H_

#include "Signal.h"
#include "SigFun.h"
#include <dronecode_sdk/telemetry.h>

namespace cdra {

  /**
   * TTI function
   * A function that takes a signal as an input
   * and computes Time-to-Intercept (TTI)
   */
  class TTIFun :  public SigFun {            

    float safeThreshold;
    float lowerx, upperx, lowery, uppery, lowerz, upperz;
    float computeTTI(float pos_east_m, float pos_north_m, float pos_down_m,
			   float vel_east_m_s, float vel_north_m_s, float vel_down_m_s) const;
    bool closeToLowerX(float pos_east_m, float pos_north_m, float pos_down_m,
                    float vel_east_m_s, float vel_north_m_s, float vel_down_m_s) const;
    bool closeToUpperX(float pos_east_m, float pos_north_m, float pos_down_m,
                    float vel_east_m_s, float vel_north_m_s, float vel_down_m_s) const;
    bool closeToLowerY(float pos_east_m, float pos_north_m, float pos_down_m,
                    float vel_east_m_s, float vel_north_m_s, float vel_down_m_s) const;
    bool closeToUpperY(float pos_east_m, float pos_north_m, float pos_down_m,
                    float vel_east_m_s, float vel_north_m_s, float vel_down_m_s) const;
    bool closeToLowerZ(float pos_up_m, float vel_up_m_s) const;
    bool closeToUpperZ(float pos_up_m, float vel_up_m_s) const;
    
  public:
    TTIFun(float lowerx, float upperx, float lowery, float uppery,
	   float lowerz, float upperz, float safeThreshold);
    virtual ~TTIFun();
    // returns the TTI at tick "t"
    float value(Signal *sig, int t);
    // returns the current TTI 
    float value(Signal *sig);
    // returns true iff current TTI within safe threshold
    bool prop(Signal *sig, int t);
    // returns true iff TTI at tick "t" within safe threshold
    bool prop(Signal *sig);
    std::string propStr() { return "tti - " + std::to_string(safeThreshold) +  " >= 0"; };
    std::string enforcer_name() { return "Boundary";};
    bool closeToXBoundary(float pos_east_m, float pos_north_m, float pos_down_m,
            float vel_east_m_s, float vel_north_m_s, float vel_down_m_s) const;
    bool closeToYBoundary(float pos_east_m, float pos_north_m, float pos_down_m,
            float vel_east_m_s, float vel_north_m_s, float vel_down_m_s) const;
    bool closeToZBoundary(float pos_up_m, float vel_up_m_s) const;

  };
  
}
#endif	/* SIGFUN_H_ */
