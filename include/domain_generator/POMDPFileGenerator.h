/*! 
 *  \author    PraveenKumar Vasudevan
 *  \date      2014
 */
#ifndef __POMDPFILEGENERATOR_H__
#define __POMDPFILEGENERATOR_H__

#include "AbstractDDLGenerator.h"

/**
 * @brief DDL generator for Cassandra POMDP format
 **/
class POMDPFileGenerator: public AbstractDDLGenerator{
public:
  /**
   * @brief Constructor
   *
   * @param stateMap State INdex map
   * @param actionMap Action index map
   * @param transitionMap Transition matrix mapping
   * @param rewardMap Reward matrix mapping
   **/
  POMDPFileGenerator(StateIndexMap& stateMap, StateIndexMap& actionMap, TransitionMap& transitionMap, RewardMap& rewardMap)
      :AbstractDDLGenerator(stateMap,actionMap,transitionMap,rewardMap){
	m_name = "POMDPFileGenerator";
    }
protected:
  /**
   * @brief Writes the body of the POMDP file
   *
   * @param fs File output stream
   * @return void
   **/
  virtual void writeBody(std::ofstream& fs);
  /**
   * @brief Writes the header info of the POMDP file
   *
   * @param fs ...
   * @return void
   **/
  virtual void writeHeader(std::ofstream& fs);
  /**
   * @brief Writes the Footer of the POMDP file
   *
   * @param fs File output stream
   * @return void
   **/
  virtual void writeFooter(std::ofstream& fs);
};

#endif