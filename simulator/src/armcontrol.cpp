
#include "armcontrol.h"
#include "Util.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

#include <math.h>
#include "Util.h"

void ArmControl::loop()
{
   Bottle* cmd = armCmdPort.read(false);
   if(status==MOVING){
      bool done=false;
      while (!done) {
	yarp::dev::ICartesianControl* iArm=iArmMap["right"];
	iArm->checkMotionDone(&done);
	yarp::os::Time::delay(0.04);   // or any suitable delay
      }
      status = REACHED;
   }else{
    if(cmd){
//       std::cout << "Got something from someone: " << cmd->get(0).asDouble()
//       << " " << cmd->get(1).asDouble() << " " << cmd->get(2).asDouble() << " " << cmd->get(3).asDouble()
//        << std::endl;
      if(cmd->size() == 4){
	double trigger = cmd->get(3).asDouble();
	if(trigger > 0){
	  std::cout << "Received new position" << std::endl;
	  Vector handPos,handOrient;
	  handPos.resize(3);
	  handPos[0] = cmd->get(0).asDouble();
	  handPos[1] = cmd->get(1).asDouble();
	  handPos[2] = cmd->get(2).asDouble();
	  //iArm->getPose(handPos, handOrient);

	  yarp::dev::ICartesianControl* iArm=iArmMap["right"];
	  iArm->goToPositionSync(handPos);
	  
	  //yarp::os::Time::delay(1.5);
	  //action = REACHED;
	}
	status = MOVING;
	actionTime = yarp::os::Time::now();
      }
    }
   }

   // TODO Should be checked for Grasping
    /*bool motionDone;   
    if(iHand->checkMotionDone(&motionDone))
    {
	if(motionDone && 
	    (yarp::os::Time::now() - actionTime) >= 3.0 )
	    status = action;
    }*/        

    if(status == REACHED){
      status = IDLE;
      Bottle &portStatus = armStatusPort.prepare();
      portStatus.clear();
      portStatus.addDouble(1.0);
      armStatusPort.write();  
      //printf("Reached\n");
    }else if(status == IDLE){
      Bottle &portStatus = armStatusPort.prepare();
      portStatus.clear();
      portStatus.addDouble(10.0);
      armStatusPort.write();  
    }else if(status == MOVING){
      Bottle &portStatus = armStatusPort.prepare();
      portStatus.clear();
      portStatus.addDouble(100.0);
      armStatusPort.write();
    }  
}

// bool ArmControl::open(yarp::os::ResourceFinder &rf)
bool ArmControl::open()
{   
    //string ctrlName;
    string robotName="icubSim";
    string remoteName;
    string localName;
    
    Json::Value robot = Config::instance()->root["robot"];
    
    if(!robot.isNull()){
      robotName = robot["name"].asString();
    }
    
    std::string partName;
    Json::Value right_arm = robot["parts"]["right_arm"];
    if(!right_arm.isNull()){
      if(right_arm["enabled"].asBool()){
	partName = "right";
	if(!configure_arm(robotName, partName)){
	  std::cout << "Failed to configure " << partName << std::endl;
	}
      }
    }
    
    Json::Value left_arm = robot["parts"]["left_arm"];
    if(!left_arm.isNull()){
      if(left_arm["enabled"].asBool()){
	partName = "left";
	if(!configure_arm(robotName, partName)){
	  std::cout << "Failed to configure " << partName << std::endl;
	}
      }
    }
    
    if(!configure_torso(robotName)){
      std::cout << "Failed to configure Torso" << std::endl;
    }
    
    //Time::turboBoost();

    // get params from the RF    
//     partName = "right";
//     // opening hand controller
//     robotName = "icubSim";
//     remoteName = string("/")+robotName+string("/")+partName+string("_arm");
//     localName = string("/armcontrol/")+partName+string("_hand");
//  
//     // open the client
//     Property option("(device remote_controlboard)");
//     option.put("remote",remoteName.c_str());
//     option.put("local",localName.c_str());
//     if (!driver.open(option)){
//       std::cout << "Remote control board cannot be opened"<<std::endl;
//         return false;
//     }
//     // open the view
//     driver.view(iHand);
//     
//     // opening arm controller
//     remoteName=string("/")+robotName+"/cartesianController/"+partName+string("_arm");
//     localName=string("/armcontrol/")+partName+string("_arm");
//     // open the client
//     Property armOption("(device cartesiancontrollerclient)");
//     armOption.put("remote",remoteName.c_str());
//     armOption.put("local",localName.c_str());
//     if (!armDriver.open(armOption))
//         return false;
// 
//     // open the view
//     armDriver.view(iArm);
//     // set trajectory time
//     iArm->setTrajTime(1.0);
// 
//     // get the torso dofs
//     Vector newDof, curDof;
//     iArm->getDOF(curDof);
//     newDof=curDof;
// 
//     // enable the torso yaw and pitch
//     // disable the torso roll
//     newDof[0]=1;
//     newDof[1]=1;
//     newDof[2]=1;
// 
//     // send the request for dofs reconfiguration
//     iArm->setDOF(newDof,curDof);
// 
//     // Dont bend more than 15 degrees
//     double min, max;
//     iArm->getLimits(0,&min,&max);
//     iArm->setLimits(0,min,15);
// 
//     // Dont turn more than 15
//     iArm->setLimits(2,-15,15);
//     iArm->setLimits(1,-15,15);
// 
// 
//     // print out some info about the controller
//     Bottle info;
//     iArm->getInfo(info);
//     std::cout << "info = " << info.toString() << "\n";
//     std::cout << "DOFs = " << curDof.toString() << " and new: "
//          << newDof.toString() << " \n";
//     iArm->setTrackingMode(false);


    action = status = IDLE;
    
    initialize_robot();

//     Vector handPos,handOrient;
//     iArm->getPose(handPos,handOrient);
//     std::cout << "Current Position: " << handPos.toString() << std::endl;
//     std::cout << "Current Orientation: " << handOrient.toString() << std::endl;
//     // Move to some initial position
//     Vector targetPos;
//     targetPos.resize(3);
//     targetPos[0] = -0.3;
//     targetPos[1] = -0.1;
//     targetPos[2] = 0.1;
//     std::cout << "Moving to initial position" << std::endl;
//     if(iArm->goToPositionSync(targetPos)){
//       std::cout << "Moved to initial position" << std::endl;
//       bool done=false;
//       while (!done) {
// 	  iArm->checkMotionDone(&done);
// 	  yarp::os::Time::delay(0.04);   // or any suitable delay
//       }
//     }else{
//       std::cout << "Cannot move to initial position" << std::endl;
//     }  
    
    bool ret=true;   
    ret = armCmdPort.open("/armcontrol/cmd/in");
    ret &= armStatusPort.open("/armcontrol/status/out");   
    return ret;
}

bool ArmControl::configure_arm(std::string& robotName, std::string& partName){
  string remoteName;
  string localName;
  remoteName = string("/")+robotName+string("/")+partName+string("_arm");
  localName = string("/armcontrol/")+partName+string("_hand");
  
  Property option("(device remote_controlboard)");
  option.put("remote",remoteName.c_str());
  option.put("local",localName.c_str());
  boost::shared_ptr<PolyDriver> driver(new PolyDriver());
  if (!driver->open(option)){
      std::cout << "Remote control board cannot be opened"<<std::endl;
      return false;
  }
  // open the view
  IPositionControl* iHand;
  driver->view(iHand);
  
  driverMap.insert(std::pair<std::string,boost::shared_ptr<PolyDriver> >(partName,driver));
  iHandMap.insert(std::pair<std::string,IPositionControl*>(partName,iHand));
  
  // opening arm controller
  remoteName=string("/")+robotName+"/cartesianController/"+partName+string("_arm");
  localName=string("/armcontrol/")+partName+string("_arm");
  // open the client
  Property armOption("(device cartesiancontrollerclient)");
  armOption.put("remote",remoteName.c_str());
  armOption.put("local",localName.c_str());
  boost::shared_ptr<PolyDriver> armDriver(new PolyDriver());
  if (!armDriver->open(armOption)){
      return false;
  }

  // open the view
  ICartesianControl* iArm;
  armDriver->view(iArm);
  
  armDriverMap.insert(std::pair<std::string,boost::shared_ptr<PolyDriver> >(partName,armDriver));
  iArmMap.insert(std::pair<std::string,ICartesianControl*>(partName,iArm));
  
  // set trajectory time
  iArm->setTrajTime(1.0);

  // get the torso dofs
  Vector newDof, curDof;
  iArm->getDOF(curDof);
  newDof=curDof;

  // enable the torso yaw and pitch
  // disable the torso roll
  newDof[0]=1;
  newDof[1]=1;
  newDof[2]=1;

  // send the request for dofs reconfiguration
  iArm->setDOF(newDof,curDof);

  // Dont bend more than 15 degrees
  double min, max;
  iArm->getLimits(0,&min,&max);
  iArm->setLimits(0,min,15);

  // Dont turn more than 15
  iArm->setLimits(2,-15,15);
  iArm->setLimits(1,-15,15);


  // print out some info about the controller
  Bottle info;
  iArm->getInfo(info);
  std::cout << "info = " << info.toString() << "\n";
  std::cout << "DOFs = " << curDof.toString() << " and new: "
      << newDof.toString() << " \n";
  iArm->setTrackingMode(false);
  return true;
}

bool ArmControl::close()
{
//     driver.close();
//     armDriver.close();
    
    for (std::map<std::string,boost::shared_ptr<PolyDriver> >::iterator it=driverMap.begin(); it!=driverMap.end(); ++it){
      it->second->close();
    }
    
    for (std::map<std::string,boost::shared_ptr<PolyDriver> >::iterator it=armDriverMap.begin(); it!=armDriverMap.end(); ++it){
      it->second->close();
    }
    
    armCmdPort.close();
    armStatusPort.close();
    return true;
}

bool ArmControl::interrupt()
{
    armCmdPort.interrupt();
    return true;
}

void ArmControl::initialize_robot(){
  Json::Value robot = Config::instance()->root["robot"];
  if(!robot.isNull()){
    for (std::map<std::string,ICartesianControl* >::iterator it=iArmMap.begin(); it!=iArmMap.end(); ++it){
      ICartesianControl* iArm = it->second;
      std::string partName = it->first;
      if(!partName.empty()){
	Json::Value armConfig = robot["parts"][it->first];
	if(!armConfig.isNull()){
	  Json::Value init = armConfig["init_pose"];
	  if(init.isArray()){
	    std::vector<double> vec;
	    if(Utils::valueToVector(init,vec)){
	      
	    }
	  }
	}
      }
    }
//     Vector handPos,handOrient;
//     iArm->getPose(handPos,handOrient);
//     std::cout << "Current Position: " << handPos.toString() << std::endl;
//     std::cout << "Current Orientation: " << handOrient.toString() << std::endl;
//     // Move to some initial position
//     Vector targetPos;
//     targetPos.resize(3);
//     targetPos[0] = -0.3;
//     targetPos[1] = -0.1;
//     targetPos[2] = 0.1;
//     std::cout << "Moving to initial position" << std::endl;
//     if(iArm->goToPositionSync(targetPos)){
//       std::cout << "Moved to initial position" << std::endl;
//       bool done=false;
//       while (!done) {
// 	  iArm->checkMotionDone(&done);
// 	  yarp::os::Time::delay(0.04);   // or any suitable delay
//       }
//     }else{
//       std::cout << "Cannot move to initial position" << std::endl;
//     }  
  }
  Json::Value right_arm = robot["parts"]["right_arm"];
  std::cout << right_arm["enabled"] << std::endl;
  
  Json::Value left_arm = robot["parts"]["left_arm"];
  std::cout << left_arm["enabled"] << std::endl;
}

bool ArmControl::configure_torso(std::string& robotName){
  /* Torso */
  string remoteName;
  string localName;
  string partName = "torso";
  remoteName = string("/")+robotName+string("/")+partName;
  localName = string("/armcontrol/")+partName;
  Property optionTorso("(device remote_controlboard)");
  optionTorso.put("remote",remoteName.c_str());
  optionTorso.put("local",localName.c_str());

  boost::shared_ptr<PolyDriver> ddTorso(new PolyDriver());
  if (!ddTorso->open(optionTorso)){
      return false;
  }
  
  // open the view
  IPositionControl* iTorso;
  ddTorso->view(iTorso);
  
  driverMap.insert(std::pair<std::string,boost::shared_ptr<PolyDriver> >(partName,ddTorso));
  iHandMap.insert(std::pair<std::string,IPositionControl*>(partName,iTorso));
  
  return true;
}

void ArmControl::move_joints(yarp::sig::Vector &qd)
{
//     bool done = false;
//     yarp::sig::Vector torsoPos;
//     torsoPos.resize(3);
//     for(int i=0 ; i < 3; ++i)
//     {
// 	torsoPos[i] = qd[i];
//     }
//     posCtrlTorso->positionMove(torsoPos.data());
//     do
//     {
// 	posCtrlTorso->checkMotionDone(&done);
// 	yarp::os::Time::delay(.01);
//     }while(!done);
// 
//     yarp::sig::Vector armPos;
//     armPos.resize(16);
//     
//     for(int i=0; i < 7; ++i)
//     {
// 	armPos[i] = qd[i+3];
//     }
//     
//     for(Json::ArrayIndex i=7; i<16; ++i)
//     {
// 	armPos[i] = Config::instance()->root["Parts"]["right_arm"]["pose_init"][i].asDouble();
//     }
//     
//     posCtrlArm->positionMove(armPos.data());
//     
//     do
//     {
// 	posCtrlArm->checkMotionDone(&done);
// 	yarp::os::Time::delay(.01);
//     }while(!done);
//     
//     std::cout << "Move to pos: "
// 	      << torsoPos.toString() << " and arm: "
// 	      << armPos.toString() << "\n";
}
