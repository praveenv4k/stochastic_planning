
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

#include <boost/shared_ptr.hpp>

#include "Config.h"

enum armstatus_t {
    IDLE,
    REACHED,
    MOVING
    };

class ArmControl
{
public:
  ArmControl(){
  }
  bool open();
  bool close();
  void loop(); 
  bool interrupt();
private:
  void initialize_robot();
  bool configure_arm(std::string& robotName,std::string& armName);
  bool configure_torso(std::string& robotName);
  void move_joints(yarp::sig::Vector &qd);
private:
  yarp::os::BufferedPort<yarp::os::Bottle> armCmdPort;
  yarp::os::BufferedPort<yarp::os::Bottle> armStatusPort;
  
//   yarp::dev::PolyDriver driver;
//   yarp::dev::IPositionControl *iHand;
//   yarp::dev::PolyDriver armDriver;
//   yarp::dev::ICartesianControl *iArm;

  std::map<std::string,yarp::dev::ICartesianControl*> iArmMap;
  std::map<std::string,yarp::dev::IPositionControl*> iHandMap;
  std::map<std::string,boost::shared_ptr<yarp::dev::PolyDriver> > driverMap;
  std::map<std::string,boost::shared_ptr<yarp::dev::PolyDriver> > armDriverMap;
  
  armstatus_t status;
  armstatus_t action;     
  double actionTime;
  std::string partName;
};

   
   



   
