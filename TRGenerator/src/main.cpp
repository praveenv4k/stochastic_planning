#include "Global.h"
#include "Trajectory.h"

void testContainer(Container<int>&);
void testDiscretizer(Container<int>&);
void testCombinator(Container<int>&,Container<int>&);

int main(void){
  Container<int> state1;
  state1.push_back(1);
  state1.push_back(2);
  testContainer(state1);
  testDiscretizer(state1);
  testCombinator(state1,state1);
  std::cout << "Hello world!";
  boost::shared_ptr<TrajectoryDiscretizer> pTrajDisc(new CircleTrajectoryDiscretizer(0,0.65,0.4,0.2));
  boost::shared_ptr<Trajectory> pTraj(new Trajectory(18*M_PI/180,pTrajDisc));
  
//   std::cout << "Trajectory Points" << std::endl;
//   pTraj->getInitPose().print();
//   for(int i=0; i<20;i++){
//     Container<double> pose = pTraj->getNextPose();
//     pose.print();
//   }
  
  std::cout << "Trajectory Points" << std::endl;
  std::vector<Container<double> > poses;
  int numPoints=20;
  if(pTraj->getAllPoses(numPoints,poses)){
    for(int i=0; i<numPoints;i++){
      Container<double> pose = poses[i];
      pose.print();
    }
  }
  return 0;
}

void testContainer(Container<int>& state){
  state.print();
}

void testDiscretizer(Container<int>& state){
  Container<double> min;
  min.resize(3);
  min[0]=-0.25;
  min[1]=-0.45;
  min[2]=-0.25;
  
  Container<double> max;
  min.resize(3);
  min[0]=-0.25;
  min[1]=-0.45;
  min[2]=-0.25;
  Container<double> step;
  Discretizer<int> discretizer(state,state,state);
  std::cout << discretizer() << std::endl;
}

void testCombinator(Container<int>& state1, Container<int>& state2){
  Combinator<int> combinator(state1);
  Container<int> state3 = combinator(state2);
  state3.print();
}