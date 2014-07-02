/*! 
 *  \author    PraveenKumar Vasudevan
 *  \date      2014
 */
#ifndef __STATEACTION_H__
#define __STATEACTION_H__

#include "Global.h"

// Dummy Class for State. Need to updated after review from Shashank
class State{
public:
  State(){
  }
  void SetState(yarp::sig::Vector posn, int getCurrentOrientTag){
  }
    bool ReachedGoal(){
      return false;
    }
    bool IsInvalidState(){
      return false;
    }
    void Invalidate(){
    }
  std::vector<double> disc;
    long int states;
};

#endif //__STATEACTION_H__