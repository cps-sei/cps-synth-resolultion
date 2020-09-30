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

#ifndef MISSIONAPP_DISTTOGROUNDFUN_H
#define MISSIONAPP_DISTTOGROUNDFUN_H

#include "Signal.h"
#include "SigFun.h"

namespace cdra {

    /**
     * Distance to Ground (DTG) function
     * A function that takes a signal as an input and computes
     * distance between the drone under control and the ground
     */
    class DTGFun :  public SigFun {

        const float safeDist;
	const float ground_z = 0; // Note: Assumes flat ground
	
        float computeDTG(float ego_z);
	
    public:
        DTGFun(float safeDist);
        virtual ~DTGFun();
        // returns the DTG at tick "t"
        float value(Signal *sig, int t);
        // returns the current DTG
        float value(Signal *sig);
        // returns true iff current DTG within safe threshold
        bool prop(Signal *sig, int t);
        // returns true iff DTG at tick "t" within safe threshold
        bool prop(Signal *sig);
        std::string propStr() { return "dist-to-ground - " + std::to_string(safeDist) +  " >= 0"; };
	std::string enforcer_name() { return "Flight";};
    };

}

#endif //MISSIONAPP_DISTTOGROUNDFUN_H
