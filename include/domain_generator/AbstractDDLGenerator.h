/*! 
 *  \author    PraveenKumar Vasudevan
 *  \date      2014
 */
#ifndef __ABSTRACTDDLGENERATOR_H__
#define __ABSTRACTDDLGENERATOR_H__

#include "Types.h"

/**
 * @brief Abstract Domain description language generator class
 **/
class AbstractDDLGenerator{
  // Protected constructor since it is an abstract class
protected:
  /**
   * @brief Constructor
   *
   * @param stateMap State Index Map
   * @param actionMap Action INdex Map
   * @param transitionMap Transition Matrix map
   * @param rewardMap Reward matrix map
   **/
  AbstractDDLGenerator(StateIndexMap& stateMap, StateIndexMap& actionMap,TransitionMap& transitionMap, RewardMap& rewardMap)
    :m_stateMap(stateMap),m_transitionMap(transitionMap),m_rewardMap(rewardMap),m_actionMap(actionMap){
  }
  /**
   * @brief Writes Header information of the DDL file
   *
   * @param fs File output stream
   * @return void
   **/
  virtual void writeHeader(std::ofstream& fs){
  }
  /**
   * @brief Writes Body of the DDL file -Derived class should override this
   *
   * @param fs ...
   * @return void
   **/
  virtual void writeBody(std::ofstream& fs){
  }
  /**
   * @brief Writes Footer information of the DDL file
   *
   * @param fs File output stream
   * @return void
   **/
  virtual void writeFooter(std::ofstream& fs){
  }
public:
  /**
   * @brief To generates the DDL file
   *
   * @param filePath File path of the DDL File
   * @return bool - true on success, false otherwise
   **/
  virtual bool generate(std::string& filePath);
  /**
   * @brief Name of the DDL
   *
   * @return :string - DDL Name
   **/
  virtual std::string getName() const {
    return m_name;
  }
protected:
  StateIndexMap m_stateMap;
  TransitionMap m_transitionMap;
  RewardMap m_rewardMap;
  StateIndexMap m_actionMap;
  std::string m_name;
};

#endif