#ifndef __TEST_H__
#define __TEST_H__

#include "Config.h"
#include "Container.h"
#include "UniformSpaceDiscretizer.h"
#include "GoalBasedSpaceDiscretizer.h"
#include "Combinator.h"
#include <boost/math/distributions.hpp>

using boost::math::normal;

#include <iostream>
  using std::cout; using std::endl; using std::left; using std::showpoint; using std::noshowpoint;
#include <iomanip>
  using std::setw; using std::setprecision;
#include <limits>
  using std::numeric_limits;

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
    std::cout << "************** End Test *****************" << std::endl;
  }
};

#endif