#ifndef __ABSTRACTDDLGENERATOR_H__
#define __ABSTRACTDDLGENERATOR_H__

#include "Types.h"

class AbstractDDLGenerator{
  // Protected constructor since it is an abstract class
protected:
  AbstractDDLGenerator(StateIndexMap& indexMap, TransitionMap& transitionMap, RewardMap& rewardMap)
    :m_indexMap(indexMap),m_transitionMap(transitionMap),m_rewardMap(rewardMap){
  }
public:
  virtual bool Generate(std::string& fileName)=0;
protected:
  StateIndexMap m_indexMap;
  TransitionMap m_transitionMap;
  RewardMap m_rewardMap;
};

#endif