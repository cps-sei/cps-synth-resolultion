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

#ifndef SIGFUN_H_
#define SIGFUN_H_

#include "Signal.h"
#include <limits>

namespace cdra {

  /**
   * Signal function 
   * A function that takes a signal as an input
   * and computes a value at some time "t" (i.e., val = f(sig, t))
   *
   * Also provides a predicate that returns true iff 
   * f(sig, t) - c >= 0, where c is some constant
   */
  class SigFun {
  protected:
    float minValue = std::numeric_limits<float>::max();
    float maxValue = std::numeric_limits<float>::min();
    float normalizedZero = 0 - minValue / maxValue - minValue;
    
  public:
    // Returns the result of applying this function to sig at tick t
    virtual float value(Signal *sig, int t);
    // Returns the result of applying this function to sig at current tick
    virtual float value(Signal *sig);
    // Returns true iff the value of the signal function at tick t
    // satisfies a certain condition (e.g., val >= c for some constant c).
    virtual bool prop(Signal *sig, int t);
    // Returns true iff the value of the signal function at current tick
    // satisfies a certain condition
    virtual bool prop(Signal *sig);
    virtual std::string propStr() { return "T"; };
    virtual std::string enforcer_name() { return "Prop"; };
    float normalizeValue(float value);
  };
  
}
#endif	/* SIGFUN_H_ */
