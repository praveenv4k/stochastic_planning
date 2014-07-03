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
  elbowEnabled = m_config["elbow"]["enabled"].asBool();
  
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
  
  {
    std::cout << "Generating Collision map file" << std::endl;
    ElapsedTime elapse(std::string("Generating the collision map File"));
    std::string checker("collision.txt");
    generateCollisionMapFile(checker);
  }
}

void DomainExtractor::generateCollisionMapFile(std::string& filePath){
  if(m_collisionMap.size()>0){
    std::fstream outStream;
    outStream.open(filePath.c_str(),std::fstream::out);
    if(outStream.good()){
      for(CollisionMap::iterator it=m_collisionMap.begin();it!=m_collisionMap.end();it++){
	outStream << it->first << " " <<  it->second << std::endl;
      }
    }
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
  
  TrajectoryDiscretizerPtr pTrajDisc = TrajectoryDiscretizerFactory::getTrajectoryDiscretizer(m_config["object"]["trajectory"]);
  
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
    for(int objId=0; objId<numPoints;objId++){
      Container<double> pose = poses[objId];
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
  
  TrajectoryDiscretizerPtr pTrajDisc = TrajectoryDiscretizerFactory::getTrajectoryDiscretizer(m_config["object"]["trajectory"]);
  
  double delta = m_config["object"]["trajectory"]["step"].asDouble();
  TrajectoryPtr pTraj(new Trajectory(delta,pTrajDisc));
    
  std::vector<Container<double> > poses;
  int numPoints=m_config["object"]["trajectory"]["samples"].asInt();
  if(!pTraj->getAllPoses(numPoints,poses)){
   std::cout << "Cannot get requested number of Points" << std::endl; 
  }
  
  std::vector<Container<double> > elbowPoses;
  if(elbowEnabled){
    std::vector<double> range;
    if(Utils::valueToVector(m_config["elbow"]["range"],range)){
      double length = m_config["elbow"]["length"].isNull() ? 30: m_config["elbow"]["length"].asDouble();
      elbowRadius = m_config["elbow"]["radius"].asDouble();
      TrajectoryDiscretizer::getElbowPoses(poses,elbowPoses,range[0],range[1],length);
    }
  }
  
  AbstractSpaceDiscretizerPtr discretizer=getSpaceDiscretizer(ss);
  
  std::cout << "State Space size: " << discretizer->size() << std::endl;
  
  int index=0;
  for(size_t i=0;i<discretizer->size();i++){
    int id = discretizer->operator()();
    Container<double> val = discretizer->getValueAtIndex(id);
    for(int objId=0; objId<numPoints;objId++){
      Container<double> pose = poses[objId];
      
      Container<double> robotPos;
      //Extract Robot position
      for(size_t posId=0;posId<m_agentDim-1;posId++){
	robotPos.push_back(val[posId]);
      }
      
      bool graspable=false;
      std::vector<double> ss = Utils::concatenate(val,pose);
      double norm = computeNorm(ss,graspable);
      
      bool collision = false;
      if(elbowPoses.size()>0 && elbowEnabled){
	Container<double> elbowPose = elbowPoses[objId];
	collision = isInCollision(robotPos,pose,elbowPose);
      }
      
      m_collisionMap[index]=collision;
      
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

bool DomainExtractor::isInCollision(Container<double> robot,Container<double> object,Container<double> elbow){
  bool ret=false;
  if(elbowEnabled){
    double dist = Utils::ptDistToLine(robot,std::pair<Container<double>,Container<double> >(object,elbow));
    ret = dist <= elbowRadius;
  }
  return ret;
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
  
  double maxReward = m_config["domain_model"]["reward"]["max"].isNull()?
			500:m_config["domain_model"]["reward"]["max"].asDouble();
			
  for(StateIndexMap::iterator it=m_stateIndexMap.begin();it!=m_stateIndexMap.end();it++){
    bool collision = m_collisionMap[it->second];
    double reward = computeReward(it->first,collision);
    m_rewardMap[it->second] = reward;
    for(StateIndexMap::iterator ait=m_actionIndexMap.begin();ait!=m_actionIndexMap.end();ait++){
      int id = ait->second;
      std::vector<double> val = ait->first;
      std::vector<double> state = it->first;
      std::vector<double> temp = state;
// Sink States
      if(collision){
	std::vector<int> nextState;
	nextState.push_back(m_stateIndexMap[state]);
	m_transitionMap[StateActionTuple(it->second,id)]=nextState;
      }
      else if(Utils::isEqual(maxReward,reward)){
	std::vector<int> nextState;
	nextState.push_back(m_stateIndexMap[state]);
	m_transitionMap[StateActionTuple(it->second,id)]=nextState;
      }
//////////////
      else{
	for(size_t j=0;j<val.size()-1;j++){
	  temp[j]+=val[j];
	}
	temp[val.size()-1]=val[val.size()-1];
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

double DomainExtractor::computeReward(std::vector<double> state,bool inCollision){
  std::vector<double> robotPos;
  std::vector<double> objectPos;
  double reward = -1;
  //Extract Robot position
  bool graspable=false;
  double norm = computeNorm(state,graspable);
  if(inCollision){
    double colReward =  m_config["domain_model"]["reward"]["collision"].isNull()?
			-50:m_config["domain_model"]["reward"]["collision"].asDouble();
    reward = -norm+colReward;
  }else{
    if(graspable){
      if(Utils::isEqual(state[m_agentDim-1],1)){
	reward = m_config["domain_model"]["reward"]["max"].isNull()?
			500:m_config["domain_model"]["reward"]["max"].asDouble();
      }else{
	//reward = 50;
	reward = -norm;
      }
    }
    else{
      reward = -norm;
    }
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
        TrajectoryDiscretizerPtr trajPtr=TrajectoryDiscretizerFactory::getTrajectoryDiscretizer(object["trajectory"]);
	ptr = AbstractSpaceDiscretizerPtr(new GoalBasedSpaceDiscretizer(trajPtr,object));
      }
    }
  }
  return ptr;
}