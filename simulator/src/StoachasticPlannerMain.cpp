// #include "Container.h"
// #include "Combinator.h"
// #include "Discretizer.h"
// #include "Types.h"

// void testContainer(Container<int>&);
// void testDiscretizer(Container<int>&);
// void testCombinator(Container<int>&,Container<int>&);
// 
// int main(void)
// {
//   Container<int> state1;
//   state1.push_back(1);
//   state1.push_back(2);
//   testContainer(state1);
//   testDiscretizer(state1);
//   testCombinator(state1,state1);
//   std::cout << "Hello world!";
//   return 0;
// }
// 
// void testContainer(Container<int>& state){
//   state.print();
// }
// 
// void testDiscretizer(Container<int>& state){
//   Discretizer<int> discretizer(state);
//   std::cout << discretizer() << std::endl;
// }
// 
// void testCombinator(Container<int>& state1, Container<int>& state2){
//   Combinator<int> combinator(state1);
//   std::cout << combinator(state2) << std::endl;
// }

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
        //period = rf.check("period", Value(5.0)).asDouble();
        return planner->open(rf);
    }

    double getPeriod( )
    {
        return 0.5;        
    }
    
    bool updateModule()
    { 
        planner->loop();
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
    // we need to initialize the drivers list 
    YARP_REGISTER_DEVICES(icubmod)

    Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf(stdout,"Error: yarp server does not seem available\n");
        return -1;
    }

    PlannerModule module;
    ResourceFinder rf;
    return module.runModule(rf);
}