
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
//#include <CGAL/Plane_3.h>

#include "Config.h"
#include "Types.h"


struct PartContext{
  std::string name;
  
  yarp::dev::IPositionControl* iPosCtrl;
  boost::shared_ptr<yarp::dev::PolyDriver> ddArm;
  
  yarp::sig::Vector init_pose;
  
  armstatus_t status;
  armstatus_t action;
  
  bool enabled;
  bool configured;
  bool initialized;
};

struct TorsoContext:public PartContext{
};

struct ArmContext:public PartContext{
  yarp::dev::ICartesianControl* iCartCtrl;
  boost::shared_ptr<yarp::dev::PolyDriver> ddCart;

  yarp::sig::Vector open_pose;
  yarp::sig::Vector close_pose;

  gstatus_t graspStatus;
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
  bool configure_arm(std::string& robotName, boost::shared_ptr<ArmContext>& ctx);
  bool configure_torso(std::string& robotName,boost::shared_ptr<TorsoContext>& ctx);
  bool move_joints(yarp::dev::IPositionControl* posCtrl, yarp::sig::Vector &qd);
  bool open_hand(boost::shared_ptr<ArmContext>& ctx);
  bool close_hand(boost::shared_ptr<ArmContext>& ctx);
  
//   inline boost::shared_ptr<PartContext>& get_context(std::string partName){
//     std::map<std::string,boost::shared_ptr<PartContext> >::iterator it = partCtxMap.find(partName);
//     if(it != partCtxMap.end()){
//       return it->second;
//     }
//     return boost::shared_ptr<PartContext>();
//   }  
private:
  yarp::os::BufferedPort<yarp::os::Bottle> armCmdPort;
  yarp::os::BufferedPort<yarp::os::Bottle> armStatusPort;
  std::map<std::string,boost::shared_ptr<PartContext> > partCtxMap;
  double actionTime;
};

   
   



   
