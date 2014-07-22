/*! 
 *  \author    PraveenKumar Vasudevan
 *  \date      2014
 */
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
#include "Types.h"

#include <iCub/ctrl/math.h>

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Geometry/Transform.h>

using namespace Eigen;
using namespace yarp::dev;

//#define SPOOF 1

/**
 * @brief Context of a Robot Part
 *        
 * The datastructure contains information about a particular part of a robot (leg,arm etc.,)
 **/
class PartContext{
public:
  /**
   * @brief Constructor
   *
   **/
  PartContext(){
    joint_speed = 10;
  }
  /**
   * @brief Destructor
   *
   **/
  virtual ~PartContext(){
  }
  /**
   * @brief Name of the part
   **/
  std::string name;
  /**
   * @brief Reference to the position control interface for this part
   **/
  yarp::dev::IPositionControl* iPosCtrl;
  /**
   * @brief Arm device driver
   **/
  boost::shared_ptr<yarp::dev::PolyDriver> ddArm;
  /**
   * @brief Initial pose of the part
   **/
  yarp::sig::Vector init_pose;
  /**
   * @brief Current pose of the part
   **/
  yarp::sig::Vector current_pose;
  /**
   * @brief Current status of the part
   **/
  armstatus_t status;
  /**
   * @brief Current action being performed by the part
   **/
  armstatus_t action;
  /**
   * @brief Joint reference speeds
   **/
  int joint_speed;
  /**
   * @brief Enabled/Disabled flag
   **/
  bool enabled;
  /**
   * @brief Configuration completion flag
   **/
  bool configured;
  /**
   * @brief Initialization completion flag
   **/
  bool initialized;  
};

/**
 * @brief Torso context
 * 
 * Data structure to manage the torso context
 **/
class TorsoContext:public PartContext{
};

/**
 * @brief Arm context
 * 
 * Data structure to manage the arm (left,right) context
 **/
class ArmContext:public PartContext{
public:
  /**
   * @brief CartesianControl interface
   **/
  yarp::dev::ICartesianControl* iCartCtrl;
  /**
   * @brief CartesianControl device driver
   **/
  boost::shared_ptr<yarp::dev::PolyDriver> ddCart;
  /**
   * @brief Hand open posture
   **/
  yarp::sig::Vector open_pose;
  yarp::sig::Vector close_pose;
  /**
   * @brief Hand closed/grasp posture
   **/
  yarp::sig::Vector init_position;
  /**
   * @brief Current world (cartesian) position
   **/
  yarp::sig::Vector curr_world_position;
  /**
   * @brief Initial world (cartesian) position
   **/
  yarp::sig::Vector init_world_position;
  /**
   * @brief Initial orientation
   **/
  yarp::sig::Vector init_orient;
  gstatus_t graspStatus;
  /**
   * @brief Grasp status
   **/
};

class ArmControl
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ArmControl(){
    
  tf1 << 0,-1.0000,0,0,
	  0,0,1.0000,0.5976,
         -1.0000,0,0,-0.0260,
	  0,0,0,1.0000;
  tf2 << 0,0,-1.0000,-0.0260,
	 -1.0000,0,0,0,
         0,1.0000,0,-0.5976,
         0,0,0,1.0000;
	
  std::cout << tf1 << std::endl;
  std::cout << tf2 << std::endl;
  
  Vector4d v1;
  v1 << -0.283779,-0.182074,-0.011660,1;
  Vector4d v2 = tf1* v1;
  Vector4d v3 = tf2* v2;
  
  std::cout << v1 << std::endl;
  std::cout << v2 << std::endl;
  std::cout << v3 << std::endl;
  
      m_joints.resize(19);

//     T =
// 
//          0   -1.0000         0         0
//          0         0    1.0000    0.5976
//    -1.0000         0         0   -0.0260
//          0         0         0    1.0000
//     /*inv T =
//          0         0   -1.0000   -0.0260
//    -1.0000         0         0         0
//          0    1.0000         0   -0.5976
//          0         0         0    1.0000*/
  }
  /**
   * @brief Initializes the ports and configures the devices
   *
   * @return bool - Success/Failure of the configuration
   **/
  bool open();
  /**
   * @brief Closes the Arm control ports
   *
   * @return bool - Success /Failure
   **/
  bool close();
  /**
   * @brief Main control loop
   *
   * @return void
   **/
  void loop(); 
  void loop2();
  /**
   * @brief Interrupt the control thread
   *
   * @return bool
   **/
  bool interrupt();
private:
  void initialize_robot();
  bool configure_arm(std::string& robotName, boost::shared_ptr<ArmContext>& ctx);
  bool configure_torso(std::string& robotName,boost::shared_ptr<TorsoContext>& ctx);
  bool move_joints(yarp::dev::IPositionControl* posCtrl, yarp::sig::Vector &qd,bool bSync=true);
  void move_joints_pos(IPositionControl* armPosCtrl,IPositionControl* torsoPosCtrl, yarp::sig::Vector &qd,bool grasp, yarp::sig::Vector& out);
  bool open_hand(boost::shared_ptr<ArmContext>& ctx,bool bSync=true);
  bool close_hand(boost::shared_ptr<ArmContext>& ctx,bool bSync=true);
  bool robot_to_world(const yarp::sig::Vector& robot,yarp::sig::Vector& world);
  bool world_to_robot(const yarp::sig::Vector& world,yarp::sig::Vector& robot);
  void make_dummy_cube(){
#if SPOOF
    if (!spoofStatePort.open("/armcontrol/spoofobject"))
    {
      std::cout <<": unable to open port to send states\n";
      return;  // unable to open; let RFModule know so that it won't run
    }
    /* Connect to iCub Simulation world */
    if(!yarp::os::Network::connect("/armcontrol/spoofobject","/icubSim/world"))
    {
      std::cout <<": unable to connect to iCub Simulation world.\n";
      return;  // unable to open; let RFModule know so that it won't run
    }
    
    yarp::os::Bottle reply;
    
//     yarp::os::Bottle del_all("world del all"); 
//     spoofStatePort.write(del_all,reply); 
    
    std::string str;
    str = "world mk sbox 0.01 0.01 0.01 -0.1 0.60 0.35 1 0 0";
    std::cout << str << std::endl;
    yarp::os::Bottle create_box(yarp::os::ConstString(str.c_str()));
    spoofStatePort.write(create_box,reply);
#endif
  }
  void set_cube_position(yarp::sig::Vector& pos){
#if SPOOF
    yarp::os::Bottle move_obj("world set sbox 1");
    yarp::os::Bottle reply;
    move_obj.addDouble(pos[0]); 
    move_obj.addDouble(pos[1]); 
    move_obj.addDouble(pos[2]);
    spoofStatePort.write(move_obj,reply);
#endif
  }
private:
#if SPOOF
  yarp::os::Port spoofStatePort;
#endif
  yarp::os::BufferedPort<yarp::os::Bottle> armCmdPort;
  yarp::os::BufferedPort<yarp::os::Bottle> armStatusPort;
  std::map<std::string,boost::shared_ptr<PartContext> > partCtxMap;
  double actionTime;
  yarp::sig::Matrix mat;
  yarp::sig::Matrix mat2;  
  Matrix4d tf1;
  Matrix4d tf2;
  yarp::sig::Vector m_joints;
};

   
   



   
