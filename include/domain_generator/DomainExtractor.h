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

class DomainExtractor{
public:
  DomainExtractor(Json::Value config):m_config(config){
  }
  void generate();
private:
  void writeStateSpace(std::ostream& stream);
  void writeActionSpace(std::ostream& stream);
  void createStateSpaceMap();
  void createActionSpaceMap();
  AbstractSpaceDiscretizerPtr getSpaceDiscretizer(Json::Value config);
  void generateTables();
  void generateDDLFile(std::string& filePath);
  double computeReward(std::vector<double> state,bool inCollision);
  double computeNorm(std::vector<double> state,bool& graspable);
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
};

#endif