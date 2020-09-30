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
#include "StlExpr.h"

namespace cdra {

  /**
   * Base class for STL Expression 
   */
  StlExpr::StlExpr() {} 
  StlExpr::~StlExpr() {}
  float StlExpr::robustness(Signal *sig, int t){
    // by default, returns 0 
    return 0;
  }
  bool StlExpr::sat(Signal *sig, int t){
    // by default, returns true
    return true;
  }

   /**
   * Atomic proposition in STL
   */
  Prop::Prop(SigFun *f) : fun(f) {}
  Prop::~Prop() {}
  float Prop::robustness(Signal *sig, int t){
    if (!sig->available(t)) return UNKNOWN_ROB;
    return fun->value(sig, t);
  }
  bool Prop::sat(Signal *sig, int t){
    if (!sig->available(t)) return UNKNOWN_SAT;	
    return fun->prop(sig, t);
  }

  /**
   * Conjunction ("AND") in STL
   */
  And::And(StlExpr *left, StlExpr *right) : left(left), right(right) {}
  And::~And() {
    delete left;
    delete right;
  }
  float And::robustness(Signal *sig, int t){
    if (!sig->available(t)) return UNKNOWN_ROB;
    return std::min(left->robustness(sig, t), right->robustness(sig, t));
  }
  /*
  float And::normalize(float value){
    return value;
  }
  */
  bool And::sat(Signal *sig, int t){
    if (!sig->available(t)) return UNKNOWN_SAT;	
    return (left->sat(sig, t) && right->sat(sig, t));
  }

  /**
   * Implication ("IMPLIES") in STL
   */
  Implies::Implies(StlExpr *left, StlExpr *right) : left(left), right(right) {}
  Implies::~Implies() {
    delete left;
    delete right;
  }
  float Implies::robustness(Signal *sig, int t){
    if (!sig->available(t)) return UNKNOWN_ROB;
    float leftRob = -1.0*left->robustness(sig, t);
    return std::max(leftRob, right->robustness(sig, t));
  }
  bool Implies::sat(Signal *sig, int t){
    if (!sig->available(t)) return UNKNOWN_SAT;		
    return (!(left->sat(sig, t)) || right->sat(sig, t));
  }
  
  /**
   * Negation ("NOT") in STL
   */
  Not::Not(StlExpr *expr) : expr(expr) {}
  Not::~Not() {
    delete expr;
  }
  float Not::robustness(Signal *sig, int t){
    if (!sig->available(t)) return UNKNOWN_ROB;
    return -(expr->robustness(sig,t));
  }
  bool Not::sat(Signal *sig, int t){
    if (!sig->available(t)) return UNKNOWN_SAT;		
    return !(expr->sat(sig, t));
  }

  /**
   * Globally ("G") in STL
   */
  Global::Global(StlExpr *expr, int begin, int end) : expr(expr), begin(begin), end(end) {}
  Global::~Global() {
    delete expr;
  }
  float Global::robustness(Signal *sig, int t){
    if (!(sig->available(t + begin) && sig->available(t + end))) return UNKNOWN_ROB;

    float min = expr->robustness(sig, t + begin);
    for (int t2 = t + begin; t2 <= t + end; t2++){
      float r = expr->robustness(sig, t2);
      if (r < min) min = r;
    }
    return min;
  }
  // Semantics of G[a,b] \phi:
  // \forall t' \in [t + begin, t + end] . (sig, t') \sat \phi
  bool Global::sat(Signal *sig, int t){
    if (!(sig->available(t + begin) && sig->available(t + end))) return UNKNOWN_SAT;	
    
    for (int t2 = t + begin; t2 <= t + end; t2++){
      if (!expr->sat(sig, t2)) return false;
    }
    return true;
  }

  
  
  /**
   * Globally ("G") in STL
   */
  PastGlobal::PastGlobal(StlExpr *expr, int begin, int end) : expr(expr), begin(begin), end(end) {}
  PastGlobal::~PastGlobal() {
    delete expr;
  }
  float PastGlobal::robustness(Signal *sig, int t){
    if (!(sig->available(t - begin) && sig->available(t - end))) return UNKNOWN_ROB;

    float min = expr->robustness(sig, t - begin);
    for (int t2 = t - begin; t2 <= t - end; t2++){
      float r = expr->robustness(sig, t2);
      if (r < min) min = r;
    }
    return min;
  }
  // Semantics of G[a,b] \phi:
  // \forall t' \in [t + begin, t + end] . (sig, t') \sat \phi
  bool PastGlobal::sat(Signal *sig, int t){
    if (!(sig->available(t - begin) && sig->available(t - end))) return UNKNOWN_SAT;
	
    for (int t2 = t - begin; t2 <= t - end; t2++){
      if (!expr->sat(sig, t2)) return false;
    }
    return true;
  }
  
  
}

