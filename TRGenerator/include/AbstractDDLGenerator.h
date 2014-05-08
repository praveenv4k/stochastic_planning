#ifndef __ABSTRACTDDLGENERATOR_H__
#define __ABSTRACTDDLGENERATOR_H__

#include "Types.h"

class AbstractDDLGenerator{
  // Protected constructor since it is an abstract class
protected:
  AbstractDDLGenerator(StateIndexMap& indexMap, StateIndexMap& actionMap,TransitionMap& transitionMap, RewardMap& rewardMap)
    :m_indexMap(indexMap),m_transitionMap(transitionMap),m_rewardMap(rewardMap),m_actionMap(actionMap){
  }
  virtual void writeHeader(std::ofstream& fs){
  }
  virtual void writeBody(std::ofstream& fs){
  }
  virtual void writeFooter(std::ofstream& fs){
  }
public:
  virtual bool generate(std::string& fileName)=0;
  virtual std::string getName() const {
    return m_name;
  }
protected:
  StateIndexMap m_indexMap;
  TransitionMap m_transitionMap;
  RewardMap m_rewardMap;
  StateIndexMap m_actionMap;
  std::string m_name;
};

#endif