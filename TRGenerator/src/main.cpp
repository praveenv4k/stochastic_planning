#include "Global.h"
#include "Trajectory.h"
#include <iostream>
#include <fstream>
#include "Action.h"

void testContainer(Container<int>&);
void testDiscretizer(Container<int>&);
void testCombinator(Container<int>&,Container<int>&);
void writeStateSpace();
void writeActionSpace();

int main(void){
  Container<int> state1;
  state1.push_back(1);
  state1.push_back(2);
  testContainer(state1);
  testDiscretizer(state1);
  testCombinator(state1,state1);
  //writeStateSpace();
  writeActionSpace();
  return 0;
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

void writeStateSpace(){
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
  
  double distThres = 0.01;
  
  boost::shared_ptr<TrajectoryDiscretizer> pTrajDisc(new CircleTrajectoryDiscretizer(0,0.65,0.4,0.2));
  boost::shared_ptr<Trajectory> pTraj(new Trajectory(18*M_PI/180,pTrajDisc));
    
  //std::cout << "Trajectory Points" << std::endl;
  std::vector<Container<double> > poses;
  int numPoints=20;
  if(!pTraj->getAllPoses(numPoints,poses)){
   std::cout << "Cannot get requested number of Points" << std::endl; 
  }
  
  std::fstream stream;
  stream.open("states.txt",std::fstream::out);
  Discretizer<double> discretizer(min,max,step);
  
  int index=0;
  for(size_t i=0;i<discretizer.size();i++){
    int id = discretizer();
    Container<double> val = discretizer.getValueAtIndex(id);
    for(int i=0; i<numPoints;i++){
      Container<double> pose = poses[i];
      double norm = Utils::computeL2norm<double>(val,pose);
      stream << index++ << " " <<  val << " " << pose << " " << 0 << " " << norm << std::endl;
    }
  }
  discretizer.reset();
  for(size_t i=0;i<discretizer.size();i++){
    int id = discretizer();
    Container<double> val = discretizer.getValueAtIndex(id);
    for(int i=0; i<numPoints;i++){
      Container<double> pose = poses[i];
      double norm = Utils::computeL2norm<double>(val,pose);
      stream << index++ << " " <<  val << " " << pose << " " << 1 << " " << norm << std::endl;
    }
  }
  stream.close();
}

void writeActionSpace()
{
#if 1
  Container<double> min;
  min.resize(4);
  min[0]=-0.02;
  min[1]=-0.02;
  min[2]=-0.02;
  min[3]=0;
  
  Container<double> max;
  max.resize(4);
  max[0]=0.02;
  max[1]=0.02;
  max[2]=0.02;
  max[3]=1;
  
  Container<double> step;
  step.resize(4);
  step[0]=0.02;
  step[1]=0.02;
  step[2]=0.02;
  step[3]=1;
#else
  Container<double> min;
  min.resize(3);
  min[0]=-0.02;
  min[1]=-0.02;
  min[2]=-0.02;
  
  Container<double> max;
  max.resize(3);
  max[0]=0.02;
  max[1]=0.02;
  max[2]=0.02;
  
  Container<double> step;
  step.resize(3);
  step[0]=0.02;
  step[1]=0.02;
  step[2]=0.02;
#endif
  
  Discretizer<double> discretizer(min,max,step);
  
  
  for(size_t i=0;i<discretizer.size();i++){
    int id = discretizer();
    Container<double> val = discretizer.getValueAtIndex(id);
    std::cout << "action" << id << " " << val << std::endl;
  }
}