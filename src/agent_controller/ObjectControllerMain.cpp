
#include "ObjectControllerModule.h"
#include <boost/math/distributions.hpp>

using boost::math::normal;

int main(void){
  Network yarp;
  if (!yarp.checkNetwork()){
    std::cout << "Error: yarp server does not seem available\n";
    return -1;
  }
#if 0
  ObjectControllerModule mod;
  ResourceFinder rf;
  return mod.runModule(rf);
#else
  ObjectController objControl(0);
  ResourceFinder rf;
  objControl.open(rf);
  std::cout << objControl.getPeriodMin() << "; " << objControl.getPeriodMean() << "; " 
            << objControl.getPeriodSigma() << "; " << objControl.getNumPoints() << std::endl;
	    
  normal s(objControl.getPeriodMean(),objControl.getPeriodSigma());
  int step = 1; // in z 
  int minRange = floor(objControl.getNumPoints()/2); // min and max z = -range to +range.
  int maxRange = -minRange+objControl.getNumPoints()-1;
  std::cout << "Standard normal distribution, mean = "<< s.mean()
  << ", standard deviation = " << s.standard_deviation() <<" Scale: " << s.scale() << std::endl;
  std::vector<double> randWait;
  double minPeriod = objControl.getPeriodMin();
  for (int z = -minRange; z <= maxRange; z++){
    double noise = pdf(s,z);
    double wait = minPeriod+noise*objControl.getPeriodScale();
    randWait.push_back(wait);
    std::cout << "Val: " << z << "Noisy : " << wait << std::endl ;
  }
  
  step = 0;
  while(true){
    if(step==objControl.getNumPoints()){
      step=0;
    }
    objControl.loop();
    double delay = randWait[step]/10;
    yarp::os::Time::delay(delay);
    //std::cout  << "Waited for : " << delay;
    step++;
  }
#endif
}
