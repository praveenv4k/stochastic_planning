#include "Generator.h"

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
  createStateSpaceMap();
  generateTables();
#endif
#endif
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

void Generator::generateTables(){
  if(m_stateIndexMap.size()==0){
    std::cout << "State map empty. call createStateSpaceMap" << std::endl;
  }
  Json::Value robot = m_config["robot"];
  Json::Value action = robot["action"];
  Container<double> min,max,step;
  Utils::valueToVector(action["min"],min);
  Utils::valueToVector(action["max"],max);
  Utils::valueToVector(action["step"],step);
  
  Discretizer<double> discretizer(min,max,step);
  std::cout << "Action Space size: " << discretizer.size() << std::endl;
    
  for(StateIndexMap::iterator it=m_stateIndexMap.begin();it!=m_stateIndexMap.end();it++){
    for(size_t i=0;i<discretizer.size();i++){
      int id = discretizer();
      Container<double> val = discretizer.getValueAtIndex(id);
      std::vector<double> state = it->first;
      std::vector<double> temp = state;
      for(size_t j=0;j<val.size()-1;j++){
	temp[j]+=val[j];
      }
      temp[val.size()-1]=val[val.size()-1];
      std::cout << temp << std::endl;
      if(m_stateIndexMap.find(temp)!=m_stateIndexMap.end()){
	std::vector<int> nextState;
	nextState.push_back(m_stateIndexMap[temp]);
	m_transitionMap[StateActionTuple(it->second,id)]=nextState;
      }
    }
    discretizer.reset();
  }
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