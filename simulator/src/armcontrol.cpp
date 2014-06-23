
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
#if 0
  for (std::map<std::string,boost::shared_ptr<PartContext> >::iterator it=partCtxMap.begin(); it!=partCtxMap.end(); ++it){
    boost::shared_ptr<PartContext> ctx = it->second;
    if(ctx!=NULL && ctx->configured){
      boost::shared_ptr<ArmContext> arm_ctx = boost::static_pointer_cast<ArmContext,PartContext>(ctx);
      if(arm_ctx!=NULL){
	if(arm_ctx->graspStatus == GRASPED)
	  open_hand(arm_ctx);
	else if(arm_ctx->graspStatus == RELEASED)
	  close_hand(arm_ctx);
      }
    }
  }
#else  
   std::map<std::string,boost::shared_ptr<PartContext> >::iterator it;
   it=partCtxMap.find(Config::instance()->root["armcontrol"]["arm"].asString());
   if(it!=partCtxMap.end()){
     boost::shared_ptr<ArmContext> arm_ctx = boost::dynamic_pointer_cast<ArmContext>(it->second);
     if(arm_ctx!=NULL){
        Bottle* cmd = armCmdPort.read(false);
	yarp::dev::ICartesianControl* iArm=arm_ctx->iCartCtrl;
	if(arm_ctx->status==MOVING){
	  bool reached;
	  bool done=false;
	  if(iArm->checkMotionDone(&done)){
	    reached = done;
	  }
	  if(arm_ctx->iPosCtrl->checkMotionDone(&done)){
	    reached&=done;
	  }
// 	  while (!done) {
// 	    iArm->checkMotionDone(&done);
// 	    yarp::os::Time::delay(0.04);   // or any suitable delay
// 	  }
	  if(reached){
	    arm_ctx->status = REACHED;
	  }
	}
	else{
	  if(cmd){
	    if(cmd->size() == 4){
	      std::cout << "Received something: " << cmd->toString() << std::endl;
	      double trigger = cmd->get(3).asDouble();
	      if(trigger > 0){
		if(trigger == 10){
		  close_hand(arm_ctx);
		}
		else if(trigger == 100){
		  open_hand(arm_ctx);
		}
	        else{
		  std::cout << "Received new position" << std::endl;
		  Vector robotPos,handOrient;
		  Vector worldPos;
		  worldPos.resize(3);
		  worldPos[0] = cmd->get(0).asDouble();
		  worldPos[1] = cmd->get(1).asDouble();
		  worldPos[2] = cmd->get(2).asDouble();
		  if(world_to_robot(worldPos,robotPos)){
#if 1
		    yarp::dev::ICartesianControl* iArm=arm_ctx->iCartCtrl;
		    iArm->goToPoseSync(robotPos,arm_ctx->init_orient);
#else
		    yarp::sig::Vector tmpQ, tmpX, tmpO;
		    tmpQ.resize(10);
		    tmpX.resize(3);
		    tmpO.resize(4);
		    const yarp::sig::Vector postn = robotPos;
		    const yarp::sig::Vector orntn = arm_ctx->init_orient;
		    if(! arm_ctx->iCartCtrl->askForPosition(arm_ctx->current_pose,postn,tmpX,tmpO,tmpQ))
			std::cout << " Inversion failed!!!\n ";

		    move_joints(arm_ctx->iPosCtrl,tmpQ,false);

		    // Update
// 		    cs.posn = tmpX;
// 		    cs.ortn = tmpO;
		    arm_ctx->current_pose = tmpQ;
#endif
		  }
		}
		arm_ctx->status = MOVING;
	        actionTime = yarp::os::Time::now();
	      }
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

      if(arm_ctx->status == REACHED){
	arm_ctx->status = IDLE;
	Bottle &portStatus = armStatusPort.prepare();
	portStatus.clear();
	portStatus.addDouble(1.0);
	armStatusPort.write();  
	//printf("Reached\n");
      }else if(arm_ctx->status == IDLE){
	Bottle &portStatus = armStatusPort.prepare();
	portStatus.clear();
	portStatus.addDouble(10.0);
	armStatusPort.write();  
      }else if(arm_ctx->status == MOVING){
	Bottle &portStatus = armStatusPort.prepare();
	portStatus.clear();
	portStatus.addDouble(100.0);
	armStatusPort.write();
      }
     }
   }
#endif
}

bool ArmControl::robot_to_world(const yarp::sig::Vector& robot,yarp::sig::Vector& world){
  if(robot.size()==3){
    Vector4d v1;
    v1 << robot[0],robot[1],robot[2],1;
    Vector4d v2 = tf1* v1;
    world.resize(3);
    world[0]=v2[0];
    world[1]=v2[1];
    world[2]=v2[2];
    return true;
  }
  return false;
}

bool ArmControl::world_to_robot(const yarp::sig::Vector& world,yarp::sig::Vector& robot){
  if(world.size()==3){
    Vector4d v1;
    v1 << world[0],world[1],world[2],1;
    Vector4d v2 = tf2* v1;
    robot.resize(3);
    robot[0]=v2[0];
    robot[1]=v2[1];
    robot[2]=v2[2];
    return true;
  }
  return false;
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
	config &= Utils::valueToVector(left_arm["open_hand"],left_ctx->open_pose);
	config &= Utils::valueToVector(left_arm["close_hand"],left_ctx->close_pose);
	
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
    
    bool ret=true;   
    ret = armCmdPort.open("/armcontrol/cmd/in");
    ret &= armStatusPort.open("/armcontrol/status/out");   
    return ret;
}

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
  //iArm->getPose();
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
    boost::shared_ptr<ArmContext> arm_ctx = boost::dynamic_pointer_cast<ArmContext>(it->second);
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
	  ctx->current_pose = ctx->init_pose;
	  boost::shared_ptr<ArmContext> arm_ctx(boost::dynamic_pointer_cast<ArmContext>(ctx));
	  if(arm_ctx!=NULL){
	    std::cout << "Moved " << partName <<  " to initial pose!"  << std::endl;
	    open_hand(arm_ctx);
	    bool done = false;
	    do{
	      arm_ctx->iPosCtrl->checkMotionDone(&done);
	      yarp::os::Time::delay(.01);
	    }while(!done);
	    yarp::os::Time::delay(2);
	    arm_ctx->iCartCtrl->getPose(arm_ctx->init_position,arm_ctx->init_orient);
	    std::cout << "Initial Position: " << arm_ctx->init_position.toString() << std::endl;
	    std::cout << "Initial Orientation: " << arm_ctx->init_orient.toString() << std::endl;
// 	    for(size_t i=arm_ctx->init_pose.size()-arm_ctx->open_pose.size();i<arm_ctx->init_pose.size();i++){
// 	      ctx->current_pose[i]=arm_ctx->open_pose[arm_ctx->open_pose.size()-i];
// 	    }
	    for(size_t i=8;i<16;i++){
	      ctx->current_pose[i]=arm_ctx->open_pose[i-8];
	    }
	    std::cout << "Current Pose: " << ctx->current_pose.toString()<< std::endl;
	  }
	}
      }
    }
  }
}

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
//       bool done=false;
//       while (!done) {
// 	ctx->iPosCtrl->checkMotionDone(&done);
// 	yarp::os::Time::delay(0.04);   // or any suitable delay
//       }
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
//       bool done=false;
//       while (!done) {
// 	ctx->iPosCtrl->checkMotionDone(&done);
// 	yarp::os::Time::delay(0.04);   // or any suitable delay
//       }
      ctx->graspStatus = RELEASED;
      bret = true;
    }
  }
  return bret;
}

bool ArmControl::move_joints(IPositionControl* posCtrl, yarp::sig::Vector &qd,bool bSync)
{
  std::cout << "Move joints : " << qd.toString() << std::endl;
    bool done = false;
    if(qd.size()>0){
      posCtrl->positionMove(qd.data());
      if(bSync){
	do{
	  posCtrl->checkMotionDone(&done);
	  yarp::os::Time::delay(.01);
	}while(!done);
      }else{
	done = true;
      }
    }
    return done;
}
