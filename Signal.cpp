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

namespace cdra {

  Signal::Signal(std::vector<std::string> signalNames) {
    std::vector<std::string>::iterator it;
    int i = 0;
    for (it = signalNames.begin(); it != signalNames.end(); ++it){
      index[*it] = i;
      i++;
    }
    // initialize the first signal vector
    std::vector<float> initSignalVector(signalNames.size());
    signal.push_back(initSignalVector);
  }

  /*
  Signal::Signal(Signal& s) {
    std::vector<std::string>::iterator it;
    index = s.index;
    signal = s.signal;
    }
  */
  
  Signal::~Signal(){}

  // Set the value of the signal at current tick to "next"
  void Signal::append(std::vector<float> next){
    signal.push_back(next);
  }
 
  // Return the current value of the named signal
  float Signal::value(std::string name){
    return signal.back()[index[name]];
  }
  
  // Return the value of the named signal at time tick "t"
  float Signal::value(std::string name, int t){
    if (signal.size() - 1 < t)
      throw "Signal unavailable!";
    return signal[t][index[name]];
  }

  bool Signal::available(int t){
    return (t >=0 && t < signal.size());
  }
  
  int Signal::length(){
    return signal.size();
  }

  void Signal::pop(){
    signal.pop_back();
  }
  
}

