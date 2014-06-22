
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
  for (std::map<std::string,boost::shared_ptr<PartContext> >::iterator it=partCtxMap.begin(); it!=partCtxMap.end(); ++it){
    boost::shared_ptr<PartContext> ctx = it->second;
    if(ctx!=NULL && ctx->configured){
      boost::shared_ptr<ArmContext> arm_ctx = boost::static_pointer_cast<ArmContext>(ctx);
      if(arm_ctx!=NULL){
	if(arm_ctx->graspStatus == GRASPED)
	  open_hand(arm_ctx);
	else if(arm_ctx->graspStatus == RELEASED)
	  close_hand(arm_ctx);
      }
    }
  }
  
//    Bottle* cmd = armCmdPort.read(false);
//    if(status==MOVING){
//       bool done=false;
//       while (!done) {
// 	yarp::dev::ICartesianControl* iArm=iCartCtrlMap["right"];
// 	iArm->checkMotionDone(&done);
// 	yarp::os::Time::delay(0.04);   // or any suitable delay
//       }
//       status = REACHED;
//    }else{
//     if(cmd){
// //       std::cout << "Got something from someone: " << cmd->get(0).asDouble()
// //       << " " << cmd->get(1).asDouble() << " " << cmd->get(2).asDouble() << " " << cmd->get(3).asDouble()
// //        << std::endl;
//       if(cmd->size() == 4){
// 	double trigger = cmd->get(3).asDouble();
// 	if(trigger > 0){
// 	  std::cout << "Received new position" << std::endl;
// 	  Vector handPos,handOrient;
// 	  handPos.resize(3);
// 	  handPos[0] = cmd->get(0).asDouble();
// 	  handPos[1] = cmd->get(1).asDouble();
// 	  handPos[2] = cmd->get(2).asDouble();
// 	  //iArm->getPose(handPos, handOrient);
// 
// 	  yarp::dev::ICartesianControl* iArm=iCartCtrlMap["right"];
// 	  iArm->goToPositionSync(handPos);
// 	  
// 	  //yarp::os::Time::delay(1.5);
// 	  //action = REACHED;
// 	}
// 	status = MOVING;
// 	actionTime = yarp::os::Time::now();
//       }
//     }
//    }
// 
//    // TODO Should be checked for Grasping
//     /*bool motionDone;   
//     if(iHand->checkMotionDone(&motionDone))
//     {
// 	if(motionDone && 
// 	    (yarp::os::Time::now() - actionTime) >= 3.0 )
// 	    status = action;
//     }*/        
// 
//     if(status == REACHED){
//       status = IDLE;
//       Bottle &portStatus = armStatusPort.prepare();
//       portStatus.clear();
//       portStatus.addDouble(1.0);
//       armStatusPort.write();  
//       //printf("Reached\n");
//     }else if(status == IDLE){
//       Bottle &portStatus = armStatusPort.prepare();
//       portStatus.clear();
//       portStatus.addDouble(10.0);
//       armStatusPort.write();  
//     }else if(status == MOVING){
//       Bottle &portStatus = armStatusPort.prepare();
//       portStatus.clear();
//       portStatus.addDouble(100.0);
//       armStatusPort.write();
//     }  
}

bool ArmControl::open()
{   
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
	bool config = true;
	boost::shared_ptr<ArmContext> right_ctx(new ArmContext());
	right_ctx->name = partName;
	right_ctx->enabled = true;
	right_ctx->status = IDLE;
	config &= Utils::valueToVector(right_arm["init_pose"],right_ctx->init_pose);
	config &= Utils::valueToVector(right_arm["open_hand"],right_ctx->close_pose);
	config &= Utils::valueToVector(right_arm["close_hand"],right_ctx->open_pose);
	
	if(config){
	  if(!configure_arm(robotName, right_ctx)){
	    std::cout << "Failed to configure " << partName << std::endl;
	  }else{
	    right_ctx->configured = true;
	    partCtxMap[partName]=right_ctx;
	  }
	}
      }
    }
    
    Json::Value left_arm = robot["parts"]["left_arm"];
    if(!left_arm.isNull()){
      if(left_arm["enabled"].asBool()){
	partName = "left";
	bool config = true;
	boost::shared_ptr<ArmContext> left_ctx(new ArmContext());
	left_ctx->name = partName;
	left_ctx->enabled = true;
	left_ctx->status = IDLE;
	config &= Utils::valueToVector(left_arm["init_pose"],left_ctx->init_pose);
	config &= Utils::valueToVector(left_arm["open_hand"],left_ctx->close_pose);
	config &= Utils::valueToVector(left_arm["close_hand"],left_ctx->open_pose);
	
	if(config){
	  if(!configure_arm(robotName, left_ctx)){
	    std::cout << "Failed to configure " << partName << std::endl;
	  }else{
	    left_ctx->configured = true;
	    partCtxMap[partName]=left_ctx;
	  }
	}
      }
    }
        
    Json::Value torso = robot["parts"]["torso"];
    if(!torso.isNull()){
      if(torso["enabled"].asBool()){
	partName = "torso";
	bool config = true;
	boost::shared_ptr<TorsoContext> torso_ctx(new TorsoContext());
	torso_ctx->name = partName;
	torso_ctx->enabled = true;
	torso_ctx->status = IDLE;
	config &= Utils::valueToVector(torso["init_pose"],torso_ctx->init_pose);
	if(config){
	  if(!configure_torso(robotName, torso_ctx)){
	    std::cout << "Failed to configure Torso" << std::endl;
	  }
	  else{
	    torso_ctx->configured = true;
	    partCtxMap[partName] = torso_ctx;
	  }
	}
      }
    }
    
    //Time::turboBoost();
    
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

// bool ArmControl::configure_arm(std::string& robotName, std::string& partName){
//   string remoteName;
//   string localName;
//   remoteName = string("/")+robotName+string("/")+partName+string("_arm");
//   localName = string("/armcontrol/")+partName+string("_hand");
//   
//   Property option("(device remote_controlboard)");
//   option.put("remote",remoteName.c_str());
//   option.put("local",localName.c_str());
//   boost::shared_ptr<PolyDriver> driver(new PolyDriver());
//   if (!driver->open(option)){
//       std::cout << "Remote control board cannot be opened"<<std::endl;
//       return false;
//   }
//   // open the view
//   IPositionControl* iHand;
//   driver->view(iHand);
//   
//   ddArmMap.insert(std::pair<std::string,boost::shared_ptr<PolyDriver> >(partName+string("_arm"),driver));
//   iPosCtrlMap.insert(std::pair<std::string,IPositionControl*>(partName+string("_arm"),iHand));
//   
//   // opening arm controller
//   remoteName=string("/")+robotName+"/cartesianController/"+partName+string("_arm");
//   localName=string("/armcontrol/")+partName+string("_arm");
//   // open the client
//   Property armOption("(device cartesiancontrollerclient)");
//   armOption.put("remote",remoteName.c_str());
//   armOption.put("local",localName.c_str());
//   boost::shared_ptr<PolyDriver> armDriver(new PolyDriver());
//   if (!armDriver->open(armOption)){
//       return false;
//   }
// 
//   // open the view
//   ICartesianControl* iArm;
//   armDriver->view(iArm);
//   
//   ddCartMap.insert(std::pair<std::string,boost::shared_ptr<PolyDriver> >(partName+string("_arm"),armDriver));
//   iCartCtrlMap.insert(std::pair<std::string,ICartesianControl*>(partName+string("_arm"),iArm));
//   
//   // set trajectory time
//   iArm->setTrajTime(1.0);
// 
//   // get the torso dofs
//   Vector newDof, curDof;
//   iArm->getDOF(curDof);
//   newDof=curDof;
// 
//   // enable the torso yaw and pitch
//   // disable the torso roll
//   newDof[0]=1;
//   newDof[1]=1;
//   newDof[2]=1;
// 
//   // send the request for dofs reconfiguration
//   iArm->setDOF(newDof,curDof);
// 
//   // Dont bend more than 15 degrees
//   double min, max;
//   iArm->getLimits(0,&min,&max);
//   iArm->setLimits(0,min,15);
// 
//   // Dont turn more than 15
//   iArm->setLimits(2,-15,15);
//   iArm->setLimits(1,-15,15);
// 
// 
//   // print out some info about the controller
//   Bottle info;
//   iArm->getInfo(info);
//   std::cout << "info = " << info.toString() << "\n";
//   std::cout << "DOFs = " << curDof.toString() << " and new: "
//       << newDof.toString() << " \n";
//   iArm->setTrackingMode(false);
//   return true;
// }

bool ArmControl::configure_arm(std::string& robotName, boost::shared_ptr<ArmContext>& ctx){
  
  if(ctx == NULL){
    return false;
  }
  
  string remoteName;
  string localName;
  remoteName = string("/")+robotName+string("/")+ctx->name+string("_arm");
  localName = string("/armcontrol/")+ctx->name+string("_hand");
  
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
  
  ctx->ddArm = driver;
  ctx->iPosCtrl = iHand;
  
  // opening arm controller
  remoteName=string("/")+robotName+"/cartesianController/"+ctx->name+string("_arm");
  localName=string("/armcontrol/")+ctx->name+string("_arm");
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
  
  ctx->ddCart = armDriver;
  ctx->iCartCtrl = iArm;
    
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
  for (std::map<std::string,boost::shared_ptr<PartContext> >::iterator it=partCtxMap.begin(); it!=partCtxMap.end(); ++it){
    it->second->ddArm->close();
    boost::shared_ptr<ArmContext> arm_ctx = boost::static_pointer_cast<ArmContext>(it->second);
    if(arm_ctx!=NULL){
      arm_ctx->ddCart->close();
    }
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
  for (std::map<std::string,boost::shared_ptr<PartContext> >::iterator it=partCtxMap.begin(); it!=partCtxMap.end(); ++it){
    boost::shared_ptr<PartContext> ctx = it->second;
    if(ctx!=NULL && ctx->configured){
      std::string partName = it->first;
      int axes;
      std::cout << "Moving " << partName <<  " to initial pose, Axes : ";
      if(ctx->iPosCtrl->getAxes(&axes)){
	std::cout << axes  << std::endl;
      }
      if(!partName.empty()){
	if(!move_joints(ctx->iPosCtrl,ctx->init_pose)){
	  std::cout << "Moving " << partName <<  " to initial pose failed!"  << std::endl;
	}else{
	  std::cout << "Moved " << partName <<  " to initial pose!"  << std::endl;
	  boost::shared_ptr<ArmContext> arm_ctx = boost::static_pointer_cast<ArmContext>(ctx);
	  if(arm_ctx!=NULL){
	    open_hand(arm_ctx);
	  }
	}
      }
    }
  }
}

// void ArmControl::initialize_robot(){
//   Json::Value robot = Config::instance()->root["robot"];
//   if(!robot.isNull()){
//     for (std::map<std::string,IPositionControl* >::iterator it=iPosCtrlMap.begin(); it!=iPosCtrlMap.end(); ++it){
//       IPositionControl* posCtrl = it->second;
//       std::string partName = it->first;
//       int axes;
//       std::cout << "Moving " << partName <<  " to initial pose, Axes : ";
//       if(posCtrl->getAxes(&axes)){
// 	std::cout << axes  << std::endl;
//       }
//       if(!partName.empty()){
// 	Json::Value armConfig = robot["parts"][it->first];
// 	if(!armConfig.isNull()){
// 	  Json::Value init = armConfig["init_pose"];
// 	  if(init.isArray()){
// 	    yarp::sig::Vector qinit;
// 	    if(Utils::valueToVector(init,qinit)){
// 	      if(!move_joints(posCtrl,qinit)){
// 		if(robot["unit"]["angle"].asString() == "deg"){
// 		  Utils::deg2Radian(qinit);
// 		}
// 		std::cout << "Moving " << partName <<  " to initial pose failed!"  << std::endl;
// 	      }
// 	    }
// 	  }
// 	}
//       }
//     }
//   }
//   
//       
// //     Vector handPos,handOrient;
// //     iArm->getPose(handPos,handOrient);
// //     std::cout << "Current Position: " << handPos.toString() << std::endl;
// //     std::cout << "Current Orientation: " << handOrient.toString() << std::endl;
// //     // Move to some initial position
// //     Vector targetPos;
// //     targetPos.resize(3);
// //     targetPos[0] = -0.3;
// //     targetPos[1] = -0.1;
// //     targetPos[2] = 0.1;
// //     std::cout << "Moving to initial position" << std::endl;
// //     if(iArm->goToPositionSync(targetPos)){
// //       std::cout << "Moved to initial position" << std::endl;
// //       bool done=false;
// //       while (!done) {
// // 	  iArm->checkMotionDone(&done);
// // 	  yarp::os::Time::delay(0.04);   // or any suitable delay
// //       }
// //     }else{
// //       std::cout << "Cannot move to initial position" << std::endl;
// //     }  
// 
// }

// bool ArmControl::configure_torso(std::string& robotName){
//   /* Torso */
//   string remoteName;
//   string localName;
//   string partName = "torso";
//   remoteName = string("/")+robotName+string("/")+partName;
//   localName = string("/armcontrol/")+partName;
//   Property optionTorso("(device remote_controlboard)");
//   optionTorso.put("remote",remoteName.c_str());
//   optionTorso.put("local",localName.c_str());
// 
//   boost::shared_ptr<PolyDriver> ddTorso(new PolyDriver());
//   if (!ddTorso->open(optionTorso)){
//       return false;
//   }
//   
//   // open the view
//   IPositionControl* iTorso;
//   ddTorso->view(iTorso);
//   
//   ddArmMap.insert(std::pair<std::string,boost::shared_ptr<PolyDriver> >(partName,ddTorso));
//   iPosCtrlMap.insert(std::pair<std::string,IPositionControl*>(partName,iTorso));
//   
//   return true;
// }

bool ArmControl::configure_torso(std::string& robotName,boost::shared_ptr<TorsoContext>& ctx){
  /* Torso */
  string remoteName;
  string localName;
  string partName = ctx->name;
  
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
  
  ctx->ddArm = ddTorso;
  ctx->iPosCtrl = iTorso;
  
  return true;
}

bool ArmControl::close_hand(boost::shared_ptr<ArmContext>& ctx)
{
  bool bret = false;
  if(ctx!=NULL && ctx->iPosCtrl!=NULL && ctx->configured){
    if(ctx->graspStatus == RELEASED){
      ctx->graspStatus = GRASPING;
      yarp::sig::Vector close_pose = ctx->close_pose;
      for(size_t i=0; (i<close_pose.size()) && (i<8); i++)
      {        
	  ctx->iPosCtrl->setRefSpeed(i+8, 80);
	  ctx->iPosCtrl->positionMove(i+8, close_pose[i]);
      }        
      ctx->graspStatus = GRASPED;
      bret = true;
    }
  }
  return bret;
}


bool ArmControl::open_hand(boost::shared_ptr<ArmContext>& ctx)
{
  bool bret = false;
  if(ctx!=NULL && ctx->iPosCtrl!=NULL && ctx->configured){
    if(ctx->graspStatus == GRASPED){
      ctx->graspStatus = GRASPING;
      yarp::sig::Vector open_pose = ctx->open_pose;
      for(size_t i=1; (i<open_pose.size()) && (i<8); i++)
      {        
	  ctx->iPosCtrl->setRefSpeed(i+8, 80);
	  ctx->iPosCtrl->positionMove(i+8, open_pose[i]);
      }        
      yarp::os::Time::delay(1.0);
      ctx->iPosCtrl->setRefSpeed(8, 30);
      ctx->iPosCtrl->positionMove(8, open_pose[0]);
      ctx->graspStatus = RELEASED;
      bret = true;
    }
  }
  return bret;
}

bool ArmControl::move_joints(IPositionControl* posCtrl, yarp::sig::Vector &qd)
{
  std::cout << "Move joints : " << qd.toString() << std::endl;
    bool done = false;
    if(qd.size()>0){
      posCtrl->positionMove(qd.data());
      do{
        posCtrl->checkMotionDone(&done);
        yarp::os::Time::delay(.01);
      }while(!done);
    }
    return done;

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
