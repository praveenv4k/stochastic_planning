/*! 
 *  \author    PraveenKumar Vasudevan
 *  \date      2014
 */
#ifndef __POMDPFILEGENERATOR_H__
#define __POMDPFILEGENERATOR_H__

#include "AbstractDDLGenerator.h"

class POMDPFileGenerator: public AbstractDDLGenerator{
public:
    POMDPFileGenerator(StateIndexMap& stateMap, StateIndexMap& actionMap, TransitionMap& transitionMap, RewardMap& rewardMap)
      :AbstractDDLGenerator(stateMap,actionMap,transitionMap,rewardMap){
	m_name = "POMDPFileGenerator";
    }
    //virtual bool generate(std::string& filePath);
protected:
    virtual void writeBody(std::ofstream& fs);
    virtual void writeHeader(std::ofstream& fs);
    virtual void writeFooter(std::ofstream& fs);
};

#endif