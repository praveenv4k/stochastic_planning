#ifndef __POMDPFILEGENERATOR_H__
#define __POMDPFILEGENERATOR_H__

#include "AbstractDDLGenerator.h"

class POMDPFileGenerator: public AbstractDDLGenerator{
public:
    POMDPFileGenerator(StateIndexMap& indexMap, StateIndexMap& actionMap, TransitionMap& transitionMap, RewardMap& rewardMap)
      :AbstractDDLGenerator(indexMap,actionMap,transitionMap,rewardMap){
	m_name = "POMDPFileGenerator";
    }
    virtual bool generate(std::string& fileName);
};

#endif