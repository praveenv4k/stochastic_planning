#include "Global.h"
#include "Trajectory.h"
#include <iostream>
#include <fstream>
#include "Action.h"
#include "Config.h"

void testContainer(Container<int>&);
void testDiscretizer(Container<int>&);
void testCombinator(Container<int>&,Container<int>&);
void testJson();
void writeStateSpace(bool writeToFile=false);
void writeActionSpace();
TrajectoryDiscretizerPtr GetTrajectoryDiscretizer(Json::Value trajConfig);

int main(void){
  Container<int> state1;
  state1.push_back(1);
  state1.push_back(2);
  testContainer(state1);
  testDiscretizer(state1);
  testCombinator(state1,state1);
  writeStateSpace();
  //writeActionSpace();
  testJson();
  return 0;
}

void testJson(){
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
    for(size_t i=0;i<min.size();i++){
      std::cout << min[i].asDouble() << " ";
    }
    std::cout << std::endl;
  }
  std::cout << robot["ss"]["dim"] << std::endl;
}

void testContainer(Container<int>& state){
  std::cout << state;
}

void testDiscretizer(Container<int>& state){
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

  Discretizer<double> discretizer(min,max,step);
  discretizer();
}

void testCombinator(Container<int>& state1, Container<int>& state2){
  Combinator<int> combinator(state1);
  Container<int> state3 = combinator(state2);
  state3.print();
}

void writeStateSpace(bool writeToFile){
  Json::Value robot = Config::instance()->root["robot"];
  Json::Value ss = robot["ss"];
  Container<double> min,max,step;
  Config::valueToVector(ss["min"],min);
  Config::valueToVector(ss["max"],max);
  Config::valueToVector(ss["step"],step);
  double distThres = robot["grasp"]["distThreshold"].asDouble();
  
  TrajectoryDiscretizerPtr pTrajDisc = GetTrajectoryDiscretizer(Config::instance()->root["object"]["trajectory"]);
  
  double delta = Config::instance()->root["object"]["trajectory"]["step"].asDouble();
  TrajectoryPtr pTraj(new Trajectory(delta,pTrajDisc));
    
  std::vector<Container<double> > poses;
  int numPoints=Config::instance()->root["object"]["trajectory"]["samples"].asInt();
  if(!pTraj->getAllPoses(numPoints,poses)){
   std::cout << "Cannot get requested number of Points" << std::endl; 
  }
  
  std::fstream stream;
  if(writeToFile){
    stream.open("states.txt",std::fstream::out);
  }
  
  Discretizer<double> discretizer(min,max,step);
  
  std::cout << "State Space size: " << discretizer.size() << std::endl;
  
  for(size_t i=0;i<discretizer.size();i++){
    int id = discretizer();
    Container<double> val = discretizer.getValueAtIndex(id);
    for(int i=0; i<numPoints;i++){
      Container<double> pose = poses[i];
      double norm = Utils::computeL2norm<double>(val,pose);
      if(writeToFile){
	stream << id << " " <<  val << " " << pose  << " " << norm << std::endl;
      }
    }
  }
//   discretizer.reset();
//   for(size_t i=0;i<discretizer.size();i++){
//     int id = discretizer();
//     Container<double> val = discretizer.getValueAtIndex(id);
//     for(int i=0; i<numPoints;i++){
//       Container<double> pose = poses[i];
//       double norm = Utils::computeL2norm<double>(val,pose);
//       if(writeToFile){
// 	stream << index++ << " " <<  val << " " << pose << " " << 1 << " " << norm << std::endl;
//       }
//     }
//   }
  if(writeToFile){
    stream.close();
  }
}

void writeActionSpace(bool writeToFile)
{
  Json::Value robot = Config::instance()->root["robot"];
  Json::Value action = robot["action"];
  Container<double> min,max,step;
  Config::valueToVector(action["min"],min);
  Config::valueToVector(action["max"],max);
  Config::valueToVector(action["step"],step);
  
  Discretizer<double> discretizer(min,max,step);
  
  std::fstream stream;
  if(writeToFile){
    stream.open("states.txt",std::fstream::out);
  }
  
  for(size_t i=0;i<discretizer.size();i++){
    int id = discretizer();
    Container<double> val = discretizer.getValueAtIndex(id);
    std::cout << "action" << id << " " << val << std::endl;
    if(writeToFile){
	stream << "action" << id << " " << val << std::endl;
    }
  }
  if(writeToFile){
    stream.close();
  }
}

TrajectoryDiscretizerPtr GetTrajectoryDiscretizer(Json::Value trajConfig){
  TrajectoryDiscretizerPtr ptr;
  if(!trajConfig.isNull()){
    std::string type = trajConfig["type"].asString();
    if(type == "circular"){
      Json::Value cirConfig = trajConfig[type.c_str()];
      if(!cirConfig.isNull()){
	Container<double> center;
	Config::valueToVector(cirConfig["center"],center);
	double radius = cirConfig["radius"].asDouble();
	double step = cirConfig["step"].asDouble();
	ptr = TrajectoryDiscretizerPtr(new CircleTrajectoryDiscretizer(center[0],center[1],center[2],radius));
      }
    }
  }
  return ptr;
}