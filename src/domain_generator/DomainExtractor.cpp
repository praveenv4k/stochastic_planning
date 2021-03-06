#include "DomainExtractor.h"
#include "POMDPFileGenerator.h"
#include "ElapsedTime.h"
#include "UniformSpaceDiscretizer.h"
#include "GoalBasedSpaceDiscretizer.h"

void DomainExtractor::generate(){
  Json::Value robotSpace = m_config["robot"]["ss"]["min"];
  m_agentDim = robotSpace.size();
  
  Json::Value objectSpace = m_config["object"]["trajectory"]["dim"];
  m_objectDim = objectSpace.asUInt();  
  
  farGrasp = m_config["robot"]["grasp"]["farGrasp"].asBool();
  
  {
    ElapsedTime elapse(std::string("Writing statespace map"));
    std::fstream stateStream;
    stateStream.open("states.txt",std::fstream::out);
    writeStateSpace(stateStream);
    stateStream.close();
  }
  
  {
    ElapsedTime elapse(std::string("Writing actionspace map"));
    std::fstream actionStream;
    actionStream.open("action.txt",std::fstream::out);
    writeActionSpace(actionStream);
    actionStream.close();
  }
  
  {
    std::cout << "Generating StateSpace Map" << std::endl;
    ElapsedTime elapse(std::string("Generating StateSpace Map"));
    createStateSpaceMap();
  }
  
  {
    std::cout << "Generating ActionSpace Map" << std::endl;
    ElapsedTime elapse(std::string("Generating ActionSpace Map"));
    createActionSpaceMap();
  }
  
  {
    std::cout << "Generating Transition and Reward Tables" << std::endl;
    ElapsedTime elapse(std::string("Generating Transition and Reward Tables"));
    generateTables();
  }
  
  {
    std::cout << "Generating DDL File" << std::endl;
    ElapsedTime elapse(std::string("Generating DDL File"));
    std::string ddlFile("domain.pomdp");
    generateDDLFile(ddlFile);
  }
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
  
  AbstractSpaceDiscretizerPtr discretizer = getSpaceDiscretizer(ss);
  
  std::cout << "State Space size: " << discretizer->size() << std::endl;
  
  int index=0;
  for(size_t i=0;i<discretizer->size();i++){
    int id = discretizer->operator()();
    Container<double> val = discretizer->getValueAtIndex(id);
    for(int i=0; i<numPoints;i++){
      Container<double> pose = poses[i];
      std::vector<double> robotPos = val;
      robotPos.pop_back();
      
      bool graspable=false;
      std::vector<double> ss = Utils::concatenate(val,pose);
      double norm = computeNorm(ss,graspable);      
      bool write=false;
      if(val[m_agentDim-1]>0){
	if(graspable || farGrasp){
	  write = true;
	}
      }else{
	write=true;
      }
      if(write){
	stream << index << " " <<  val << " " << pose  << " " << norm << std::endl;
	index++;
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
  
  AbstractSpaceDiscretizerPtr discretizer=getSpaceDiscretizer(ss);
  
  std::cout << "State Space size: " << discretizer->size() << std::endl;
  
  int index=0;
  for(size_t i=0;i<discretizer->size();i++){
    int id = discretizer->operator()();
    Container<double> val = discretizer->getValueAtIndex(id);
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
      if(val[m_agentDim-1]>0){
	if(graspable || farGrasp){
	  m_stateIndexMap[ss]=index++;
	}
      }else{
	m_stateIndexMap[ss]=index++;
      }
    }
  }
  std::cout << m_stateIndexMap.size() <<std::endl;
}

void DomainExtractor::createActionSpaceMap(){
  Json::Value robot = m_config["robot"];
  Json::Value action = robot["action"];
  Container<double> min,max,step;
  Utils::valueToVector(action["min"],min);
  Utils::valueToVector(action["max"],max);
  Utils::valueToVector(action["step"],step);
  
  AbstractSpaceDiscretizerPtr discretizer = getSpaceDiscretizer(action);  
  for(size_t i=0;i<discretizer->size();i++){
    int id = discretizer->operator()();
    Container<double> val = discretizer->getValueAtIndex(id);
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
	nextState.push_back(found->second);
	m_transitionMap[StateActionTuple(it->second,id)]=nextState;
      }
      else{
	std::vector<int> nextState;
	nextState.push_back(m_stateIndexMap[state]);
	m_transitionMap[StateActionTuple(it->second,id)]=nextState;
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
  if(graspable){
    if(Utils::isEqual(state[m_agentDim-1],1)){
      reward = 500;
    }else{
      //reward = 50;
      reward = -norm;
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
  
  AbstractSpaceDiscretizerPtr discretizer=getSpaceDiscretizer(action);
  std::cout << "Action Space size: " << discretizer->size() << std::endl;
    
  for(size_t i=0;i<discretizer->size();i++){
    int id = discretizer->operator()();
    Container<double> val = discretizer->getValueAtIndex(id);
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
    else if(type == "linear"){
      Json::Value linConfig = trajConfig[type.c_str()];
      if(!linConfig.isNull()){
	Container<double> start,end;
	Utils::valueToVector(linConfig["start"],start);
	Utils::valueToVector(linConfig["end"],end);
	ptr = TrajectoryDiscretizerPtr(new LinearTrajectoryDiscretizer(start,end));
      }
    }
    else{
    }
  }
  return ptr;
}

AbstractSpaceDiscretizerPtr DomainExtractor::getSpaceDiscretizer(Json::Value config){
  AbstractSpaceDiscretizerPtr ptr;
  if(!config.isNull()){
    Json::Value disc = config["discretizer"];
    if(!disc.isNull()){
      std::string name = disc.asString();
      if(name == "uniform"){
	Container<double> min,max,step;
	Utils::valueToVector(config["min"],min);
	Utils::valueToVector(config["max"],max);
	Utils::valueToVector(config["step"],step);
	ptr = AbstractSpaceDiscretizerPtr(new UniformSpaceDiscretizer<double>(min,max,step));
      }
      else if(name == "goal"){
	Json::Value object = m_config["object"];
        TrajectoryDiscretizerPtr trajPtr=getTrajectoryDiscretizer(object["trajectory"]);
	ptr = AbstractSpaceDiscretizerPtr(new GoalBasedSpaceDiscretizer(trajPtr,object));
      }
    }
  }
  return ptr;
}