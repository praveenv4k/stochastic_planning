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
  TrajectoryDiscretizerPtr getTrajectoryDiscretizer(Json::Value trajConfig);
  void generateTables();
private:
  Json::Value m_config;
  StateIndexMap m_stateIndexMap;
  TransitionMap m_transitionMap;
};

#endif