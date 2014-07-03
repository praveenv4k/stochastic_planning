/*! 
 *  \author    PraveenKumar Vasudevan
 *  \date      2014
 */
#ifndef __DOMAINEXTRACTOR_H__
#define __DOMAINEXTRACTOR_H__

#include <fstream>
#include "Container.h"
#include "Combinator.h"
#include "AbstractSpaceDiscretizer.h"
#include "Types.h"
#include "States.h"
#include "Action.h"
#include "Trajectory.h"
#include "json/json.h"

/**
 * @brief Domain model extractor
 **/
class DomainExtractor{
public:
  /**
   * @brief Constructor
   *
   * @param config JSON Configuration
   **/
  DomainExtractor(Json::Value config):m_config(config){
  }
  /**
   * @brief Generate the model
   *
   * @return void
   **/
  void generate();
private:
  /**
   * @brief Writes the State space info to a file
   *
   * @param stream File output stream
   * @return void
   **/
  void writeStateSpace(std::ostream& stream);
  /**
   * @brief Writes action space info to a file
   *
   * @param stream File output stream
   * @return void
   **/
  void writeActionSpace(std::ostream& stream);
  /**
   * @brief Creates a state space mapping
   *
   * @return void
   **/
  void createStateSpaceMap();
  /**
   * @brief Creates an action space mapping
   *
   * @return void
   **/
  void createActionSpaceMap();
  /**
   * @brief Composes the Space discretizer pointer from the name specified in the configuration information
   *
   * @param config JSON config
   * @return AbstractSpaceDiscretizerPtr - Returns a pointer to the abstract space discretizer
   **/
  AbstractSpaceDiscretizerPtr getSpaceDiscretizer(Json::Value config);
  /**
   * @brief Generates the Transition and Reward tables
   *
   * @return void
   **/
  void generateTables();
  /**
   * @brief Generates the Domain Description Language File
   *
   * @param filePath File path
   * @return void
   **/
  void generateDDLFile(std::string& filePath);
  /**
   * @brief Generates the Collision mapping file
   *
   * @param filePath File path
   * @return void
   **/
  void generateCollisionMapFile(std::string& filePath);
  /**
   * @brief Computes the reward given the augmented state space
   *
   * @param state Augmented State
   * @param inCollision Flag to specify the collision status
   * @return double Return the reward value depending on the scheme specified in the Config File
   **/
  double computeReward(std::vector<double> state,bool inCollision);
  /**
   * @brief Computes the norm of the vector
   *
   * @param state Augmented state 
   * @param graspable Graspable flag set when the distance is less than the threshold specified in the config file
   * @return double Distance norm
   **/
  double computeNorm(std::vector<double> state,bool& graspable);
  /**
   * @brief Given robot,object and elbow position at a given instance, it will tell if it is in collision space
   *
   * @param robot Position of the robot
   * @param object Position of the object
   * @param elbow Position of the elbow
   * @return bool - True if collision,false otherwise
   **/
  bool isInCollision(Container<double> robot,Container<double> object,Container<double> elbow);
private:
  Json::Value m_config;
  StateIndexMap m_stateIndexMap;
  StateIndexMap m_actionIndexMap;
  TransitionMap m_transitionMap;
  CollisionMap m_collisionMap;
  RewardMap m_rewardMap;
  size_t m_agentDim;
  size_t m_objectDim;
  bool farGrasp;
  bool elbowEnabled;
  double elbowRadius;
  std::map<int,bool> m_goodStates;
  std::map<int,bool> m_badStates;
};

#endif