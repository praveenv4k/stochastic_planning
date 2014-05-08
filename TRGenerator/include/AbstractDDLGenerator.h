#ifndef __ABSTRACTDDLGENERATOR_H__
#define __ABSTRACTDDLGENERATOR_H__

#include "Types.h"

class AbstractDDLGenerator{
  // Protected constructor since it is an abstract class
  protected:
    AbstractDDLGenerator(StateIndexMap& indexMap, TransitionMap& transitionMap, RewardMap& rewardMap);
  public:
    virtual void Generate()=0;
};

#endif