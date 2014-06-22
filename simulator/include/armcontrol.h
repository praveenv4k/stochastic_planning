
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
  bool move_joints(yarp::dev::IPositionControl* posCtrl, yarp::sig::Vector &qd);
  bool open_hand(std::string hand);
  bool close_hand(std::string hand);
  
  inline yarp::dev::IPositionControl* get_pos_ctrl(std::string partName){
    yarp::dev::IPositionControl* pos_ctrl = NULL;
    std::map<std::string,yarp::dev::IPositionControl*>::iterator it = iPosCtrlMap.find(partName);
    if(it != iPosCtrlMap.end()){
      pos_ctrl = it->second;
    }
    return pos_ctrl;
  }
  
  inline yarp::dev::ICartesianControl* get_cart_ctrl(std::string partName){
    yarp::dev::ICartesianControl* cart_ctrl = NULL;
    std::map<std::string,yarp::dev::ICartesianControl*>::iterator it = iCartCtrlMap.find(partName);
    if(it != iCartCtrlMap.end()){
      cart_ctrl = it->second;
    }
    return cart_ctrl;
  }
  
private:
  yarp::os::BufferedPort<yarp::os::Bottle> armCmdPort;
  yarp::os::BufferedPort<yarp::os::Bottle> armStatusPort;
  
  std::map<std::string,yarp::dev::ICartesianControl*> iCartCtrlMap;
  std::map<std::string,yarp::dev::IPositionControl*> iPosCtrlMap;
  std::map<std::string,boost::shared_ptr<yarp::dev::PolyDriver> > ddArmMap;
  std::map<std::string,boost::shared_ptr<yarp::dev::PolyDriver> > ddCartMap;
  
  armstatus_t status;
  armstatus_t action;     
  double actionTime;
  std::string partName;
};

   
   



   
