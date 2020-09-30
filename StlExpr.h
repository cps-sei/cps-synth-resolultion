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

#ifndef STLEXPR_H_
#define STLEXPR_H_

#include "Signal.h"
#include "SigFun.h"

namespace cdra {

  /**
   * Base class for STL Expression 
   */
  class StlExpr {
  protected:
    const bool  UNKNOWN_SAT = 1;
    const float UNKNOWN_ROB = 0;     
  public:
    StlExpr();
    virtual ~StlExpr();
    virtual float robustness(Signal *sig, int t);
    virtual bool sat(Signal *sig, int t);
    virtual std::string exprStr() {return "T";};
    virtual std::string generalStr() { return "Propname"; };
  };

  /**
   * Atomic proposition in STL
   */
  class Prop : public StlExpr {
    SigFun *fun;
  public:
    Prop(SigFun *f);
    virtual ~Prop();
    float robustness(Signal *sig, int t);
    float normalize(float value);
    bool sat(Signal *sig, int t);
    std::string exprStr() { return "Prop(" + fun->propStr() + ")";};
    std::string generalStr() { return fun->enforcer_name(); };
  };

  /**
   * Conjunction ("AND") in STL
   */
  class And : public StlExpr {
    StlExpr *left, *right;
  public:
    And(StlExpr *left, StlExpr *right);
    virtual ~And();
    float robustness(Signal *sig, int t);
    bool sat(Signal *sig, int t);
    std::string exprStr() {
      return "(" + left->exprStr() + ") AND (" + right->exprStr() + ")";};
  };

  /**
   * Negation ("NOT") in STL
   */
  class Not : public StlExpr {
    StlExpr *expr;
  public:
    Not(StlExpr *expr);
    virtual ~Not();
    float robustness(Signal *sig, int t);
    bool sat(Signal *sig, int t);
    std::string exprStr() { return "!(" + expr->exprStr() + ")";};
  };

  
  /**
   * Globally ("G") in STL
   */
  class Global : public StlExpr {
    StlExpr *expr;   
    int begin, end; // begin & end time bound
  public:
    Global(StlExpr *expr, int begin, int end);
    virtual ~Global();
    float robustness(Signal *sig, int t);
    bool sat(Signal *sig, int t);
    std::string exprStr() {
      return "G_[t+" + std::to_string(begin) + ",t+" + std::to_string(end) + "]("
	+ expr->exprStr() + ")";};
  };

   /**
   * Implies ("=>") in STL
   */
  class Implies : public StlExpr {
    StlExpr *left, *right;   
  public:
    Implies(StlExpr *left, StlExpr *right);
    virtual ~Implies();
    float robustness(Signal *sig, int t);
    bool sat(Signal *sig, int t);
    std::string exprStr() { return "(" + left->exprStr() + ") => (" + right->exprStr() + ")";};
  };

    /**
   * Past globally ("PG") in STL
   */
  class PastGlobal : public StlExpr {
    StlExpr *expr;   
    int begin, end; // begin & end time bound
  public:
    PastGlobal(StlExpr *expr, int begin, int end);
    virtual ~PastGlobal();
    float robustness(Signal *sig, int t);
    bool sat(Signal *sig, int t);
    std::string exprStr() { return "PG_[t-" + std::to_string(begin) + ",t-" + std::to_string(end) + "](" +
	expr->exprStr() + ")";};
  };

  
  
}

#endif	/* STLEXPR_H_ */
