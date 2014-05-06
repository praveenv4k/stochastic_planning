#ifndef __TEST_H__
#define __TEST_H__

#include "Config.h"
#include "Container.h"
#include "Discretizer.h"

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

    Discretizer<double> discretizer(min,max,step);
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
      for(size_t i=0;i<min.size();i++){
	std::cout << min[i].asDouble() << " ";
      }
      std::cout << std::endl;
    }
    std::cout << robot["ss"]["dim"] << std::endl;
  }
  
  static void testAll(){
    Container<int> state1;
    state1.push_back(1);
    state1.push_back(2);
    // Test functions
    testContainer(state1);
    testDiscretizer(state1);
    testCombinator(state1,state1);
    testJson();
  }
};

#endif