
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

#include <iCub/ctrl/math.h>

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Geometry/Transform.h>

//using namespace Eigen::Matrix4d;
using namespace Eigen;
//using Eigen::
using namespace yarp::dev;

/**
 * @brief Context of a Robot Part
 *        
 * The datastructure contains information about a particular part of a robot (leg,arm etc.,)
 **/
class PartContext{
public:
  PartContext(){
    joint_speed = 10;
  }
  virtual ~PartContext(){
  }
  std::string name;
  
  yarp::dev::IPositionControl* iPosCtrl;
  boost::shared_ptr<yarp::dev::PolyDriver> ddArm;
  
  yarp::sig::Vector init_pose;
  yarp::sig::Vector current_pose;
  
  armstatus_t status;
  armstatus_t action;
  int joint_speed;
  
  bool enabled;
  bool configured;
  bool initialized;  
};

class TorsoContext:public PartContext{
};

class ArmContext:public PartContext{
public:
  yarp::dev::ICartesianControl* iCartCtrl;
  boost::shared_ptr<yarp::dev::PolyDriver> ddCart;

  yarp::sig::Vector open_pose;
  yarp::sig::Vector close_pose;

  yarp::sig::Vector init_position;
  yarp::sig::Vector curr_world_position;
  yarp::sig::Vector init_world_position;
  yarp::sig::Vector init_orient;

  gstatus_t graspStatus;
};

class ArmControl
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ArmControl(){
    
#if 0
    mat.resize(4,4);
    
    yarp::sig::Vector r1;r1.resize(4);r1[0]=0;r1[1]=-1;r1[2]=0;r1[3]=0;
    yarp::sig::Vector r2;r2.resize(4);r2[0]=0;r2[1]=0;r2[2]=1;r2[3]=0.5976;
    yarp::sig::Vector r3;r3.resize(4);r3[0]=-1;r3[1]=0;r3[2]=0;r3[3]=-0.0260;
    yarp::sig::Vector r4;r4.resize(4);r4[0]=0;r4[1]=0;r4[2]=0;r4[3]=1;
    
    mat.setRow(0,r1);
    mat.setRow(1,r2);
    mat.setRow(2,r3);
    mat.setRow(3,r4);
    
    std::cout << mat.toString() << std::endl;
    
    mat2.resize(4,4);
    
    //     /*inv T =
//          0         0   -1.0000   -0.0260
//    -1.0000         0         0         0
//          0    1.0000         0   -0.5976
//          0         0         0    1.0000*/
    
    yarp::sig::Vector r11;r11.resize(4);r11[0]=0;r11[1]=0;r11[2]=-1;r11[3]=-0.0260;
    yarp::sig::Vector r21;r21.resize(4);r21[0]=-1;r21[1]=0;r21[2]=0;r21[3]=0;
    yarp::sig::Vector r31;r31.resize(4);r31[0]=0;r31[1]=1;r31[2]=0;r31[3]=-0.5976;
    yarp::sig::Vector r41;r41.resize(4);r41[0]=0;r41[1]=0;r41[2]=0;r41[3]=1;
    
    mat2.setRow(0,r11);
    mat2.setRow(1,r21);
    mat2.setRow(2,r31);
    mat2.setRow(3,r41);
    
    std::cout << mat.toString() << std::endl;
#else
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
//   transform.matrix() = mat1;
#endif
    
    //-0.283779 -0.182074 -0.011660 1
    
//     yarp::sig::Vector p1;p1.resize(4);p1[0]=-0.283779;p1[1]=-0.182074;p1[2]=-0.011660;p1[3]=1;
//     yarp::sig::Vector p2;p2.resize(4);
//     p2[0]=iCub::ctrl::dot(mat.getRow(0),p1);
//     p2[1]=iCub::ctrl::dot(mat.getRow(1),p1);
//     p2[2]=iCub::ctrl::dot(mat.getRow(2),p1);
//     p2[3]=iCub::ctrl::dot(mat.getRow(3),p1);
//     
//     yarp::sig::Vector p3;p3.resize(4);
//     p3[0]=iCub::ctrl::dot(mat2.getRow(0),p2);
//     p3[1]=iCub::ctrl::dot(mat2.getRow(1),p2);
//     p3[2]=iCub::ctrl::dot(mat2.getRow(2),p2);
//     p3[3]=iCub::ctrl::dot(mat2.getRow(3),p2);
    
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
  bool open();
  bool close();
  void loop(); 
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
  yarp::sig::Matrix mat;
  yarp::sig::Matrix mat2;
  
  Matrix4d tf1;
  Matrix4d tf2;
//   Transform<double,4,Affine> transform;
};

   
   



   
