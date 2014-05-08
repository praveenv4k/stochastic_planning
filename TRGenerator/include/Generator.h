#ifndef __GENERATOR_H__
#define __GENERATOR_H__

#include <fstream>
#include "Container.h"
#include "Combinator.h"
#include "Discretizer.h"
#include "Types.h"
#include "States.h"
#include "Action.h"
#include "Trajectory.h"
#include "json/json.h"

class Generator{
public:
  Generator(Json::Value config):m_config(config){
  }
  void generate();
private:
  void writeStateSpace(std::ostream& stream);
  void writeActionSpace(std::ostream& stream);
  void createStateSpaceMap();
  void createActionSpaceMap();
  TrajectoryDiscretizerPtr getTrajectoryDiscretizer(Json::Value trajConfig);
  void generateTables();
  double computeReward(std::vector<double> state);
private:
  Json::Value m_config;
  StateIndexMap m_stateIndexMap;
  StateIndexMap m_actionIndexMap;
  TransitionMap m_transitionMap;
  RewardMap m_rewardMap;
  size_t m_agentDim;
  size_t m_objectDim;
};

#endif