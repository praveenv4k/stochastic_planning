#ifndef __TEST_H__
#define __TEST_H__

#include "Config.h"
#include "Container.h"
#include "UniformSpaceDiscretizer.h"
#include "GoalBasedSpaceDiscretizer.h"
#include "Combinator.h"

class Test{
public:
  static void testContainer(Container<int>& state){
    std::cout << state;
  }
  
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
  
  static void testCombinator(Container<int>& state1, Container<int>& state2){
    Combinator<int> combinator(state1);
    Container<int> state3 = combinator(state2);
    state3.print();
  }
  
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
    std::cout << "************** End Test *****************" << std::endl;
  }
};

#endif