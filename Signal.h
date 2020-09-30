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

#ifndef SIGNAL_H_
#define SIGNAL_H_

#include <string>
#include <vector>
#include <iterator>
#include <map>

namespace cdra {

  /**
   * STL signal 
   * A function from time to a set of named values
   */
  class Signal {            
    
  protected:
    const float INIT_SIGNAL_VAL = 0;
    // Mapping from signal name to integer index
    std::map<std::string, int> index;
    // Signal is a sequence of vectors of index values
    std::vector<std::vector<float>> signal;
    
  public:
    Signal(std::vector<std::string> signalNames);
    virtual ~Signal();
    // Set the value of the signal at current tick to "next"
    void append(std::vector<float> next);
    // Remove the last element from the signal
    void pop();
    // Return the current value of the named signal
    float value(std::string name);
    // Return the value of the named signal at time tick "t"
    float value(std::string name, int t);
    bool available(int t);
    int length();
    
  };
  
}
#endif	/* SIGNAL_H_ */
