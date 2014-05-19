
#include <string>
#include <vector>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Rand.h>
#include <yarp/os/Time.h>


class Planner
{
   
public:

    Planner()
    {
      first = true;
    }

    bool open(yarp::os::ResourceFinder &rf);

    bool close();
    void loop(); 
    bool interrupt();

private:
    yarp::os::BufferedPort<yarp::os::Bottle> plannerCmdPort;
    yarp::os::BufferedPort<yarp::os::Bottle> plannerStatusPort;
    
    double t;
    double t0;
    double t1;
    bool first;
};

   
   



   
