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

#ifndef MISSIONAPP_DISTFUN_H
#define MISSIONAPP_DISTFUN_H

#include "Signal.h"
#include "SigFun.h"

namespace cdra {

    /**
     * Distance to Target (DTT) function
     * A function that takes a signal as an input and computes
     * distance between the drone under control (target) and an enemy drone
     */
    class DTTFun :  public SigFun {

        const float safeDist;
	
        float computeDTT(float x1, float y1, float z1, float x2, float y2, float z2) const;
    public:
        DTTFun(float safeDist);
        virtual ~DTTFun();
        // returns the DTT at tick "t"
        float value(Signal *sig, int t);
        // returns the current DTT
        float value(Signal *sig);
        // returns true iff current DTT within safe threshold
        bool prop(Signal *sig, int t);
        // returns true iff DTT at tick "t" within safe threshold
        bool prop(Signal *sig);
        std::string propStr() { return "dist-to-target - " + std::to_string(safeDist) +  " >= 0"; };
	std::string enforcer_name() { return "Runaway";};
    };

}

#endif //MISSIONAPP_DISTFUN_H
