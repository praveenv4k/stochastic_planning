#ifndef __POMDPFILEGENERATOR_H__
#define __POMDPFILEGENERATOR_H__

#include "AbstractDDLGenerator.h"

class POMDPFileGenerator: public AbstractDDLGenerator{
public:
    POMDPFileGenerator(StateIndexMap& indexMap, TransitionMap& transitionMap, RewardMap& rewardMap);
    virtual void Generate();
};

#endif