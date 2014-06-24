#include "DomainExtractor.h"
#include "POMDPFileGenerator.h"

void DomainExtractor::generate(){
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
  Json::Value robotSpace = m_config["robot"]["ss"]["min"];
  m_agentDim = robotSpace.size();
  
  Json::Value objectSpace = m_config["object"]["trajectory"]["dim"];
  m_objectDim = objectSpace.asUInt();  
  
  std::fstream stateStream;
  stateStream.open("states.txt",std::fstream::out);
  writeStateSpace(stateStream);
  stateStream.close();
  
  std::fstream actionStream;
  actionStream.open("action.txt",std::fstream::out);
  writeActionSpace(actionStream);
  actionStream.close();

  
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

void DomainExtractor::generateDDLFile(std::string& filePath){
  POMDPFileGenerator gen(m_stateIndexMap,m_actionIndexMap,m_transitionMap,m_rewardMap);
  gen.generate(filePath);
}
  
void DomainExtractor::writeStateSpace(std::ostream& stream){
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
      
      bool graspable=false;
      std::vector<double> ss = Utils::concatenate(val,pose);
      double norm = computeNorm(ss,graspable);      
      bool write=false;
      if(val[m_agentDim-1]>0){
	if(graspable){
	  write = true;
	}
      }else{
	write=true;
      }
      //write=true;
      if(write){
	index++;
	stream << index << " " <<  val << " " << pose  << " " << norm << std::endl;
      }
    }
  }
}

void DomainExtractor::createStateSpaceMap(){
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
      
      std::vector<double> robotPos;
      //Extract Robot position
      for(size_t i=0;i<m_agentDim-1;i++){
	robotPos.push_back(val[i]);
      }
      
      bool graspable=false;
      std::vector<double> ss = Utils::concatenate(val,pose);
      double norm = computeNorm(ss,graspable);
      //double distThres = m_config["robot"]["grasp"]["distThreshold"].asDouble();
      if(val[m_agentDim-1]>0){
	if(graspable){
	  //std::vector<double> ss = Utils::concatenate(val,pose);
	  m_stateIndexMap[ss]=index++;
	}
      }else{
	//std::vector<double> ss = Utils::concatenate(val,pose);
	m_stateIndexMap[ss]=index++;
      }
    }
  }
  std::cout << m_stateIndexMap.size() <<std::endl;
  //Utils::SaveMap("statesmap.txt",m_stateIndexMap);
}

void DomainExtractor::createActionSpaceMap(){
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

void DomainExtractor::generateTables(){
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

double DomainExtractor::computeNorm(std::vector<double> state,bool& graspable){
  std::vector<double> robotPos;
  std::vector<double> objectPos;
  for(size_t i=0;i<m_agentDim-1;i++){
    robotPos.push_back(state[i]);
  }
  //Extract Object Position
  for(size_t i=m_agentDim;i<m_agentDim+m_objectDim;i++){
    objectPos.push_back(state[i]);
  }
  double radius = m_config["object"]["radius"].asDouble();
  objectPos[1] = objectPos[1]+ radius;
  double norm = Utils::computeL2norm(robotPos,objectPos);
  double distThres = m_config["robot"]["grasp"]["distThreshold"].asDouble();
  graspable = (norm <= distThres);
  return norm;
}


double DomainExtractor::computeReward(std::vector<double> state){
  std::vector<double> robotPos;
  std::vector<double> objectPos;
  double reward = -1;
  //Extract Robot position
  bool graspable=false;
  double norm = computeNorm(state,graspable);
//   double distThres = m_config["robot"]["grasp"]["distThreshold"].asDouble();
  //if(Utils::isEqual(norm,distThres)){
  if(graspable){
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

void DomainExtractor::writeActionSpace(std::ostream& stream)
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
    stream << id << " " << val << std::endl;
  }
}

TrajectoryDiscretizerPtr DomainExtractor::getTrajectoryDiscretizer(Json::Value trajConfig){
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