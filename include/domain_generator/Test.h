/*! 
 *  \author    PraveenKumar Vasudevan
 *  \date      2014
 */
#ifndef __TEST_H__
#define __TEST_H__

#include "Config.h"
#include "Container.h"
#include "UniformSpaceDiscretizer.h"
#include "GoalBasedSpaceDiscretizer.h"
#include "Combinator.h"
#include <boost/math/distributions.hpp>
#include "Types.h"
#include "POMDPFileGenerator.h"

using boost::math::normal;

#include <iostream>
  using std::cout; using std::endl; using std::left; using std::showpoint; using std::noshowpoint;
#include <iomanip>
  using std::setw; using std::setprecision;
#include <limits>
  using std::numeric_limits;

  /**
   * @brief Unit test class
   **/
  class Test{
public:
  /**
   * @brief Test the container class
   *
   * @param state Container object
   * @return void
   **/
  static void testContainer(Container<int>& state){
    std::cout << state;
  }
  
  /**
   * @brief Discretizer test method
   *
   * @param state Container object
   * @return void
   **/
  static void testDiscretizer(Container<int>& state){
    Container<double> min;
    min.resize(3);
    min[0]=-0.25;
    min[1]=0.45;
    min[2]=0.25;
    
    Container<double> max;
    max.resize(3);
    max[0]=0.25;
    max[1]=0.85;
    max[2]=0.55;
    
    Container<double> step;
    step.resize(3);
    step[0]=0.02;
    step[1]=0.02;
    step[2]=0.02;

    UniformSpaceDiscretizer<double> discretizer(min,max,step);
    discretizer();
  }
  
  /**
   * @brief Combinator class test method
   *
   * @param state1 Container object 1
   * @param state2 Container object 2
   * @return void
   **/
  static void testCombinator(Container<int>& state1, Container<int>& state2){
    Combinator<int> combinator(state1);
    Container<int> state3 = combinator(state2);
    state3.print();
  }
  
  /**
   * @brief Test JSON parsing and access
   *
   * @return void
   **/
  static void testJson(){
    Config* config = Config::instance();
    Json::Value root = config->root;
    Json::Value::Members members = root.getMemberNames();
    for(size_t i=0;i<members.size();i++){
      std::cout<<members[i] << std::endl;
    }
    Json::Value robot = config->root["robot"];
    std::cout << robot["name"].asString() << std::endl;
    Json::Value min = robot["ss"]["min"];
    if(min.isArray()){
      for(Json::ArrayIndex i=0;i<min.size();i++){
	std::cout << min[i].asDouble() << " ";
      }
      std::cout << std::endl;
    }
    std::cout << robot["ss"]["dim"] << std::endl;
  }
  
  /**
   * @brief Test linear discretizer
   *
   * @return void
   **/
  static void testLinearDiscretizer(){
    Container<double> v1;v1.resize(3);v1[0]=0;v1[1]=0;v1[2]=2;
    Container<double> v2;v2.resize(3);v2[0]=2;v2[1]=0;v2[2]=4;
    LinearTrajectoryDiscretizer disc(v1,v2);
    std::vector<Container<double> > poses;
    disc.getAllPoses(10,poses);
    for(int i=0;i<10;i++){
      std::cout << poses[i] << std::endl;
    }
  }
  
  /**
   * @brief Test goal based discretizer
   *
   * @return void
   **/
  static void testGoalBasedDiscretizer(){
    Json::Value object = Config::instance()->root["object"];
    Json::Value linConfig = object["trajectory"]["linear"];
    TrajectoryDiscretizerPtr ptr;
    if(!linConfig.isNull()){
      Container<double> start,end;
      Utils::valueToVector(linConfig["start"],start);
      Utils::valueToVector(linConfig["end"],end);
      ptr = TrajectoryDiscretizerPtr(new LinearTrajectoryDiscretizer(start,end));
    }
    if(ptr!=NULL){
      GoalBasedSpaceDiscretizer disc(ptr,object);
    }
  }
  
  /**
   * @brief Test the point distance calculation utility
   *
   * @return void
   **/
  static void testPointDistance(){
    Container<double> pt;pt.resize(3);pt[0]=10;pt[1]=23;pt[2]=9;
    Container<double> p1;p1.resize(3);p1[0]=1;p1[1]=2;p1[2]=2;
    Container<double> p2;p2.resize(3);p2[0]=20;p2[1]=5;p2[2]=3;
    
    double dist = Utils::ptDistToLine(pt,std::pair<Container<double>,Container<double> >(p1,p2));
    
    std::cout << "Distance of the point " << pt << " from line (" << p1 << ");(" << p2 << ") : " << dist << std::endl;
    
    pt[0]=1;pt[1]=1;pt[2]=1;
    p1[0]=0;p1[1]=0;p1[2]=0;
    p2[0]=0;p2[1]=0;p2[2]=1;
    
    dist = Utils::ptDistToLine(pt,std::pair<Container<double>,Container<double> >(p1,p2));
    
    std::cout << "Distance of the point " << pt << " from line (" << p1 << ");(" << p2 << ") : " << dist << std::endl;
    
    pt[0]=0;pt[1]=0;pt[2]=1;
    p1[0]=0;p1[1]=0;p1[2]=0;
    p2[0]=0;p2[1]=0;p2[2]=1;
    
    dist = Utils::ptDistToLine(pt,std::pair<Container<double>,Container<double> >(p1,p2));
    
    std::cout << "Distance of the point " << pt << " from line (" << p1 << ");(" << p2 << ") : " << dist << std::endl;
  }
  
  /**
   * @brief Test normal distribution class
   *
   * @return void
   **/
  static void testNormalDistribution(){
    normal s;
    double step = 1.; // in z 
    double range = 4; // min and max z = -range to +range.
    int precision = 17; // traditional tables are only computed to much lower precision.

    std::cout << "Standard normal distribution, mean = "<< s.mean()
    << ", standard deviation = " << s.standard_deviation() << std::endl;
    
    std::cout << "Probability distribution function values" << std::endl;
    std::cout << "  z " "      pdf " << std::endl;
    std::cout.precision(5);
    for (double z = -range; z < range + step; z += step)
    {
      std::cout << left << setprecision(3) << setw(6) << z << " " 
	<< setprecision(precision) << setw(12) << pdf(s, z) << std::endl;
    }
    std::cout.precision(6); // default
  }
  
  static AbstractSpaceDiscretizerPtr getSpaceDiscretizer(Json::Value config){
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
	Json::Value object = Config::instance()->root["object"];
        TrajectoryDiscretizerPtr trajPtr=TrajectoryDiscretizerFactory::getTrajectoryDiscretizer(object["trajectory"]);
	ptr = AbstractSpaceDiscretizerPtr(new GoalBasedSpaceDiscretizer(trajPtr,object));
      }
    }
  }
  return ptr;
}
  
  static void testWriteStateActionSpace(){
    std::fstream tstream;
    tstream.open("testtrans.txt",std::fstream::out);
    {
      std::fstream stream;
      std::fstream rstream;
      stream.open("teststates.txt",std::fstream::out);
      rstream.open("testreward.txt",std::fstream::out);
      Json::Value robot = Config::instance()->root["robot"];
      Json::Value ss = robot["ss"];
      Container<double> min,max,step;
      Utils::valueToVector(ss["min"],min);
      Utils::valueToVector(ss["max"],max);
      Utils::valueToVector(ss["step"],step);
      double distThres = robot["grasp"]["distThreshold"].asDouble();
      
      TrajectoryDiscretizerPtr pTrajDisc = TrajectoryDiscretizerFactory::getTrajectoryDiscretizer(Config::instance()->root["object"]["trajectory"]);
      
      double delta = Config::instance()->root["object"]["trajectory"]["step"].asDouble();
      TrajectoryPtr pTraj(new Trajectory(delta,pTrajDisc));
	
      std::vector<Container<double> > poses;
      int numPoints=Config::instance()->root["object"]["trajectory"]["samples"].asInt();
      if(!pTraj->getAllPoses(numPoints,poses)){
      std::cout << "Cannot get requested number of Points" << std::endl; 
      }
      
      AbstractSpaceDiscretizerPtr discretizer =  getSpaceDiscretizer(ss);
      
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

	  double diff = robotPos[0]-pose[0];
	  double norm = diff*diff;
	  if(norm == 0){
	    norm = 500;
	  }
	  else{
	    norm = (-norm-5);
	  }
	  bool write=true;
	  if(write){
	    stream << index << " " <<  val << " " << pose  << " " << norm << std::endl;
	    //R: * :0 : * -26.4267
	    rstream << "R: * :" << index  << " : * " << norm << std::endl;
	    for(int act=0;act<4;act++){
	      //T: 1 : 241 : 241 1
	      tstream << "T: " << act << " : " << index  <<  " : " << index << " " << 1 << std::endl;
	    }
	    tstream << std::endl;
	    index++;
	  }
	}
      }
    }
    {
      std::fstream stream;
      stream.open("testactions.txt",std::fstream::out);
      Json::Value robot = Config::instance()->root["robot"];
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
  }
  
  
  static void createStateSpaceMap(StateIndexMap& m_stateIndexMap){
  Json::Value robot = Config::instance()->root["robot"];
  Json::Value ss = robot["ss"];
  Container<double> min,max,step;
  Utils::valueToVector(ss["min"],min);
  Utils::valueToVector(ss["max"],max);
  Utils::valueToVector(ss["step"],step);
  double distThres = robot["grasp"]["distThreshold"].asDouble();
  
  TrajectoryDiscretizerPtr pTrajDisc = TrajectoryDiscretizerFactory::getTrajectoryDiscretizer(Config::instance()->root["object"]["trajectory"]);
  
  double delta = Config::instance()->root["object"]["trajectory"]["step"].asDouble();
  TrajectoryPtr pTraj(new Trajectory(delta,pTrajDisc));
    
  std::vector<Container<double> > poses;
  int numPoints=Config::instance()->root["object"]["trajectory"]["samples"].asInt();
  if(!pTraj->getAllPoses(numPoints,poses)){
   std::cout << "Cannot get requested number of Points" << std::endl; 
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
      for(size_t posId=0;posId<val.size()-1;posId++){
	robotPos.push_back(val[posId]);
      }
      
      bool graspable=false;
      std::vector<double> ss = Utils::concatenate(val,pose);
      
      
      m_stateIndexMap[ss]=index++;
    }
  }
  std::cout << m_stateIndexMap.size() <<std::endl;
}

static void createActionSpaceMap(StateIndexMap& m_actionIndexMap){
  Json::Value robot = Config::instance()->root["robot"];
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

static void generateTables(StateIndexMap& m_stateIndexMap,StateIndexMap& m_actionIndexMap,TransitionMap& m_transitionMap,RewardMap& m_rewardMap){
  if(m_stateIndexMap.size()==0 || m_actionIndexMap.size()==0){
    std::cout << "State/Action map empty. call createStateSpaceMap" << std::endl;
  }
  
  double maxReward = Config::instance()->root["domain_model"]["reward"]["max"].isNull()?
			500:Config::instance()->root["domain_model"]["reward"]["max"].asDouble();
			
  for(StateIndexMap::iterator it=m_stateIndexMap.begin();it!=m_stateIndexMap.end();it++){
    double diff = it->first[0]-it->first[1];
    double norm = diff*diff;
    if(norm == 0){
      norm = 500;
    }
    else{
      norm = (-norm-5);
    }
    double reward = norm;
    m_rewardMap[it->second] = reward;
    for(StateIndexMap::iterator ait=m_actionIndexMap.begin();ait!=m_actionIndexMap.end();ait++){
      int id = ait->second;
      std::vector<double> val = ait->first;
      std::vector<double> state = it->first;
      std::vector<double> temp = state;
      if(Utils::isEqual(maxReward,reward)){
	std::vector<int> nextState;
	int nxtStateId = m_stateIndexMap[state];
	nextState.push_back(nxtStateId);
	m_transitionMap[StateActionTuple(it->second,id)]=nextState;
      }
      else{
// 	for(size_t j=0;j<val.size()-1;j++){
// 	  temp[j]+=val[j];
// 	}
	if(it->first[0]==1 ||it->first[0]==2){
	  if(ait->second==0){
	    temp[0]+=1;
	  }else if(ait->second==1){
	    temp[0]-=1;
	  }
	}else if(it->first[0]==3){
	  if(ait->second==1){
	    temp[0]-=1;
	  }else if(ait->second==2){
	    temp[0]+=1;
	  }
	}else{
	  if(ait->second==2){
	    temp[0]+=1;
	  }else if(ait->second==3){
	    temp[0]-=1;
	  }
	}
	//temp[val.size()-1]=val[val.size()-1];
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
  
  /**
   * @brief Master test method
   *
   * @return void
   **/
  static void testAll(){
    std::cout << "************** Begin Test ***************" << std::endl;
    Container<int> state1;
    state1.push_back(1);
    state1.push_back(2);
    // Test functions
    testContainer(state1);
    testDiscretizer(state1);
    testCombinator(state1,state1);
    testJson();
    testLinearDiscretizer();
    testGoalBasedDiscretizer();
    testPointDistance();
    testNormalDistribution();
    testWriteStateActionSpace();
    StateIndexMap states,actions;
    RewardMap reward;
    TransitionMap transition;
    createStateSpaceMap(states);
    createActionSpaceMap(actions);
    generateTables(states,actions,transition,reward);
    std::string ddlFile("domain.pomdp");
    POMDPFileGenerator gen(states,actions,transition,reward);
    gen.generate(ddlFile);
    std::cout << "************** End Test *****************" << std::endl;
  }
};

#endif