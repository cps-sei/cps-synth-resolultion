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

#ifndef MISSIONAPP_RECONFUN_H
#define MISSIONAPP_RECONFUN_H

#include "Signal.h"
#include "SigFun.h"

namespace cdra {

    /**
     * Distance to Elevation (DTE) function
     * A function that takes a signal as an input and computes
     * distance between the drone under control and the ideal elevation
     */
    class ReconFun :  public SigFun {

        const float goal_z;
	const float acceptable_range;
	const float lowerx, lowery, upperx, uppery;
	
        float computeDTE(float ego_z);
	bool isInReconZone(float ego_x, float ego_y);
	
    public:
        ReconFun(float goal_z, float acceptable_range, float lowerx,
		 float lowery, float upperx, float uppery);
        virtual ~ReconFun();
        // returns the DTE at tick "t"
        float value(Signal *sig, int t);
        // returns the current DTE
        float value(Signal *sig);
        // returns true iff current DTE is within acceptable_range
        bool prop(Signal *sig, int t);
        // returns true iff DTE at tick "t" is within acceptable_range
        bool prop(Signal *sig);
	std::string enforcer_name() { return "Missile";};
        std::string propStr() { return "dist-to-elevation - " + std::to_string(acceptable_range) +  " >= 0"; };
    };

}

#endif //MISSIONAPP_RECONFUN_H
