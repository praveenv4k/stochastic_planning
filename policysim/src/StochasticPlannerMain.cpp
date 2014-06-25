#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>

#include <gsl/gsl_math.h>

#include <stdio.h>

#include "planner.h"

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

class PlannerModule: public RFModule
{
   Planner* planner;

public:

    PlannerModule()
    {
        //period = 1.0;
        planner = new Planner();
    }

    ~PlannerModule()
    {
        delete planner;        
    }


    bool configure(ResourceFinder &rf)
    {
#if 1
      planner->read_actions("action.txt");
      planner->read_states("states.txt");
      planner->read_policy("domain.pomdpx","domain.policy");
      return true;
#else
      bool ret=false;
        //period = rf.check("period", Value(5.0)).asDouble();
      if(planner->open(rf)){
	Network yarp;
	if(! Network::connect("/armcontrol/status/out","/planner/cmd/in")){
	  std::cout <<": unable to connect to arm control output port.\n";
	}
	else{
         if(! Network::connect("/planner/status/out","/armcontrol/cmd/in")){
          std::cout <<": unable to connect to arm control input port.\n";
         }else{
	   ret=true;
	 }   
       }
      }
      return ret;
#endif
    }

    double getPeriod( )
    {
        return 0.5;        
    }
    
    bool updateModule()
    { 
//         planner->loop();
        return true; 
    }

    bool interruptModule()
    {
        fprintf(stderr, "Interrupting\n");
        planner->interrupt();
        return true;
    }

    bool close()
    {
        fprintf(stderr, "Calling close\n");
        planner->close();
        return true;
    }
};

int main(int argc, char *argv[])
{   
#if 0
  // we need to initialize the drivers list 
    YARP_REGISTER_DEVICES(icubmod)

    Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf(stdout,"Error: yarp server does not seem available\n");
        return -1;
    }
    
    std::string policyFile;
    std::string domainFile;
    std::string statesFile;
    std::string actionFile;
#endif
    PlannerModule module;
    ResourceFinder rf;
    return module.runModule(rf);
}