#include "Generator.h"
#include "POMDPFileGenerator.h"

void Generator::generate(){
#if 0
  writeStateSpace(std::cout);
  writeActionSpace(std::cout);
#else
#if 0
  std::fstream stateStream;
  stateStream.open("states.txt",std::fstream::out);
  writeStateSpace(stateStream);
  stateStream.close();
  
  std::fstream actionStream;
  actionStream.open("action.txt",std::fstream::out);
  writeActionSpace(actionStream);
  actionStream.close();
#else
//   std::fstream stateStream;
//   stateStream.open("states.txt",std::fstream::out);
//   writeStateSpace(stateStream);
//   stateStream.close();

  Json::Value robotSpace = m_config["robot"]["ss"]["min"];
  m_agentDim = robotSpace.size();
  
  Json::Value objectSpace = m_config["object"]["trajectory"]["dim"];
  m_objectDim = objectSpace.asUInt();  
  
  std::cout << "Generating StateSpace Map" << std::endl;
  createStateSpaceMap();
  std::cout << "Generating ActionSpace Map" << std::endl;
  createActionSpaceMap();
  std::cout << "Generating Transition and Reward Tables" << std::endl;
  generateTables();
  std::cout << "Generating DDL File" << std::endl;
  std::string ddlFile("domain.pomdp");
  generateDDLFile(ddlFile);
#endif
#endif
}

void Generator::generateDDLFile(std::string& filePath){
  POMDPFileGenerator gen(m_stateIndexMap,m_actionIndexMap,m_transitionMap,m_rewardMap);
  gen.generate(filePath);
}
  
void Generator::writeStateSpace(std::ostream& stream){
  Json::Value robot = m_config["robot"];
  Json::Value ss = robot["ss"];
  Container<double> min,max,step;
  Utils::valueToVector(ss["min"],min);
  Utils::valueToVector(ss["max"],max);
  Utils::valueToVector(ss["step"],step);
  double distThres = robot["grasp"]["distThreshold"].asDouble();
  
  TrajectoryDiscretizerPtr pTrajDisc = getTrajectoryDiscretizer(m_config["object"]["trajectory"]);
  
  double delta = m_config["object"]["trajectory"]["step"].asDouble();
  TrajectoryPtr pTraj(new Trajectory(delta,pTrajDisc));
    
  std::vector<Container<double> > poses;
  int numPoints=m_config["object"]["trajectory"]["samples"].asInt();
  if(!pTraj->getAllPoses(numPoints,poses)){
   std::cout << "Cannot get requested number of Points" << std::endl; 
  }
  
  Discretizer<double> discretizer(min,max,step);
  
  std::cout << "State Space size: " << discretizer.size() << std::endl;
  
  int index=0;
  for(size_t i=0;i<discretizer.size();i++){
    int id = discretizer();
    Container<double> val = discretizer.getValueAtIndex(id);
    for(int i=0; i<numPoints;i++){
      Container<double> pose = poses[i];
      std::vector<double> robotPos = val;
      robotPos.pop_back();
      double norm = Utils::computeL2norm<double>(robotPos,pose);
      stream << index++ << " " <<  val << " " << pose  << " " << norm << std::endl;
    }
  }
}

void Generator::createStateSpaceMap(){
  Json::Value robot = m_config["robot"];
  Json::Value ss = robot["ss"];
  Container<double> min,max,step;
  Utils::valueToVector(ss["min"],min);
  Utils::valueToVector(ss["max"],max);
  Utils::valueToVector(ss["step"],step);
  double distThres = robot["grasp"]["distThreshold"].asDouble();
  
  TrajectoryDiscretizerPtr pTrajDisc = getTrajectoryDiscretizer(m_config["object"]["trajectory"]);
  
  double delta = m_config["object"]["trajectory"]["step"].asDouble();
  TrajectoryPtr pTraj(new Trajectory(delta,pTrajDisc));
    
  std::vector<Container<double> > poses;
  int numPoints=m_config["object"]["trajectory"]["samples"].asInt();
  if(!pTraj->getAllPoses(numPoints,poses)){
   std::cout << "Cannot get requested number of Points" << std::endl; 
  }
  
  Discretizer<double> discretizer(min,max,step);
  
  std::cout << "State Space size: " << discretizer.size() << std::endl;
  
  int index=0;
  for(size_t i=0;i<discretizer.size();i++){
    int id = discretizer();
    Container<double> val = discretizer.getValueAtIndex(id);
    for(int i=0; i<numPoints;i++){
      Container<double> pose = poses[i];
      std::vector<double> ss = Utils::concatenate(val,pose);
      m_stateIndexMap[ss]=index++;
    }
  }
  std::cout << m_stateIndexMap.size() <<std::endl;
  //Utils::SaveMap("statesmap.txt",m_stateIndexMap);
}

void Generator::createActionSpaceMap(){
  Json::Value robot = m_config["robot"];
  Json::Value action = robot["action"];
  Container<double> min,max,step;
  Utils::valueToVector(action["min"],min);
  Utils::valueToVector(action["max"],max);
  Utils::valueToVector(action["step"],step);
  
  Discretizer<double> discretizer(min,max,step);
    
  for(size_t i=0;i<discretizer.size();i++){
    int id = discretizer();
    Container<double> val = discretizer.getValueAtIndex(id);
    m_actionIndexMap[val]=id;
  }
}

void Generator::generateTables(){
  if(m_stateIndexMap.size()==0 || m_actionIndexMap.size()==0){
    std::cout << "State/Action map empty. call createStateSpaceMap" << std::endl;
  }
    
  for(StateIndexMap::iterator it=m_stateIndexMap.begin();it!=m_stateIndexMap.end();it++){
    m_rewardMap[it->second] = computeReward(it->first);
    for(StateIndexMap::iterator ait=m_actionIndexMap.begin();ait!=m_actionIndexMap.end();ait++){
      int id = ait->second;
      std::vector<double> val = ait->first;
      std::vector<double> state = it->first;
      std::vector<double> temp = state;
      for(size_t j=0;j<val.size()-1;j++){
	temp[j]+=val[j];
      }
      temp[val.size()-1]=val[val.size()-1];
      //std::cout << temp << std::endl;
      StateIndexMap::iterator found = m_stateIndexMap.find(temp);
      if(found!=m_stateIndexMap.end()){
	std::vector<int> nextState;
	//nextState.push_back(m_stateIndexMap[temp]);
	nextState.push_back(found->second);
	m_transitionMap[StateActionTuple(it->second,id)]=nextState;
	//m_rewardMap[StateActionTuple(it->second,id)] = computeReward(temp);
      }
      else{
	std::vector<int> nextState;
	nextState.push_back(m_stateIndexMap[state]);
	m_transitionMap[StateActionTuple(it->second,id)]=nextState;
	//m_rewardMap[StateActionTuple(it->second,id)] = computeReward(state);
      }
    }
  }
}

double Generator::computeReward(std::vector<double> state){
  std::vector<double> robotPos;
  std::vector<double> objectPos;
  double reward = -1;
  //Extract Robot position
  for(size_t i=0;i<m_agentDim-1;i++){
    robotPos.push_back(state[i]);
  }
  //Extract Object Position
  for(size_t i=m_agentDim;i<m_agentDim+m_objectDim;i++){
    objectPos.push_back(state[i]);
  }
  double norm = Utils::computeL2norm(robotPos,objectPos);
  double distThres = m_config["robot"]["grasp"]["distThreshold"].asDouble();
  if(Utils::isEqual(norm,distThres)){
    if(Utils::isEqual(state[m_agentDim-1],1)){
      reward = 500;
    }else{
      reward = 50;
    }
  }
  else{
    reward = -norm;
  }
  return reward;
}

void Generator::writeActionSpace(std::ostream& stream)
{
  Json::Value robot = m_config["robot"];
  Json::Value action = robot["action"];
  Container<double> min,max,step;
  Utils::valueToVector(action["min"],min);
  Utils::valueToVector(action["max"],max);
  Utils::valueToVector(action["step"],step);
  
  Discretizer<double> discretizer(min,max,step);
  std::cout << "Action Space size: " << discretizer.size() << std::endl;
    
  for(size_t i=0;i<discretizer.size();i++){
    int id = discretizer();
    Container<double> val = discretizer.getValueAtIndex(id);
    stream << "action" << id << " " << val << std::endl;
  }
}

TrajectoryDiscretizerPtr Generator::getTrajectoryDiscretizer(Json::Value trajConfig){
  TrajectoryDiscretizerPtr ptr;
  if(!trajConfig.isNull()){
    std::string type = trajConfig["type"].asString();
    if(type == "circular"){
      Json::Value cirConfig = trajConfig[type.c_str()];
      if(!cirConfig.isNull()){
	Container<double> center;
	Utils::valueToVector(cirConfig["center"],center);
	double radius = cirConfig["radius"].asDouble();
	ptr = TrajectoryDiscretizerPtr(new CircleTrajectoryDiscretizer(center[0],center[1],center[2],radius));
      }
    }
    else if(type== "circular2d"){
      Json::Value cirConfig = trajConfig[type.c_str()];
      if(!cirConfig.isNull()){
	Container<double> center;
	Utils::valueToVector(cirConfig["center"],center);
	double radius = cirConfig["radius"].asDouble();
	ptr = TrajectoryDiscretizerPtr(new Circle2DTrajectoryDiscretizer(center[0],center[1],radius));
      }
    }
    else{
    }
  }
  return ptr;
}