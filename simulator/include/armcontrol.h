
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

enum armstatus_t {
    IDLE,
    REACHED,
    MOVING
    };

class ArmControl
{
   
public:

    ArmControl()
    {
      first = true;
       // constructor
    }

//     bool open(yarp::os::ResourceFinder &rf);
    bool open();
    bool close();
    void loop(); 
    bool interrupt();

private:
    yarp::os::BufferedPort<yarp::os::Bottle> armCmdPort;
    yarp::os::BufferedPort<yarp::os::Bottle> armStatusPort;
    yarp::dev::PolyDriver driver;
    yarp::dev::IPositionControl *iHand;
    yarp::dev::PolyDriver armDriver;
    yarp::dev::ICartesianControl *iArm;

    armstatus_t status;
    armstatus_t action;     
    double actionTime;
    std::string partName;
    bool first;
};

   
   



   
