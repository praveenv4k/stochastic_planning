
#include "armcontrol.h"
#include "Utils.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

#include <math.h>

void ArmControl::loop()
{  
#if 0 // GRASP CHECK
  for (std::map<std::string,boost::shared_ptr<PartContext> >::iterator it=partCtxMap.begin(); it!=partCtxMap.end(); ++it){
    boost::shared_ptr<PartContext> ctx = it->second;
    if(ctx!=NULL && ctx->configured){
      boost::shared_ptr<ArmContext> arm_ctx = boost::dynamic_pointer_cast<ArmContext>(ctx);
      if(arm_ctx!=NULL){
	if(arm_ctx->graspStatus == GRASPING || arm_ctx->graspStatus==RELEASING){
	  bool done;
	  if(arm_ctx->iPosCtrl->checkMotionDone(&done)){
	    if(done){
	      if(arm_ctx->graspStatus==GRASPING){
		std::cout << "Grasp Complete! " << std::endl;
		arm_ctx->graspStatus= GRASPED;
	      }
	      else if(arm_ctx->graspStatus==RELEASING){
		std::cout << "Open Complete! " << std::endl;
		arm_ctx->graspStatus= RELEASED;
	      }
	    }
	  }
	}
	else{
	  if(arm_ctx->graspStatus == GRASPED){
	    std::cout << "Opening Hand! " << std::endl;
	    open_hand(arm_ctx,false);
	  }
	  else if(arm_ctx->graspStatus == RELEASED){
	    std::cout << "Closing Hand! " << std::endl;
	    close_hand(arm_ctx,false);
	  }
	}
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
#if 0
	  bool reached;
	  bool done=false;
	  if(iArm->checkMotionDone(&done)){
	    reached = done;
	  }
// 	  if(arm_ctx->iPosCtrl->checkMotionDone(&done)){
// 	    reached&=done;
// 	  }
	  //std::cout << "Reached Target: " << reached << std::endl;
	  if(reached){
	    std::cout << "Reached Target: " << std::endl;
	    arm_ctx->status = REACHED;
	  }
#else
	if(arm_ctx->graspStatus == GRASPING || arm_ctx->graspStatus==RELEASING){
	  std::cout << "Grasp Check! " << std::endl;
	  bool done;
	  if(arm_ctx->iPosCtrl->checkMotionDone(&done)){
	    if(done){
	      if(arm_ctx->graspStatus==GRASPING){
		std::cout << "Grasp Complete! " << std::endl;
		arm_ctx->graspStatus= GRASPED;
	      }
	      else if(arm_ctx->graspStatus==RELEASING){
		std::cout << "Open Complete! " << std::endl;
		arm_ctx->graspStatus= RELEASED;
	      }
	    }
	  }
	}else{
	  arm_ctx->status = arm_ctx->action;
	}
#endif
	}
	else{
	  if(cmd){
	    if(cmd->size() == 4){
	      std::cout << "Received something: " << cmd->toString() << std::endl;
	      double trigger = cmd->get(3).asDouble();
	      if(trigger > 0){
		if(trigger == 10){
		  std::cout << "Closing Hand: " << cmd->toString() << std::endl;
		  if(arm_ctx->graspStatus == RELEASED)
		    close_hand(arm_ctx,false);
		  //close_hand(arm_ctx);
		}
		else if(trigger == 100){
		  std::cout << "Opening Hand: " << cmd->toString() << std::endl;
		  if(arm_ctx->graspStatus == GRASPED)
		    open_hand(arm_ctx,false);
		  //open_hand(arm_ctx);
		}
	        else{
		  Vector robotPos,handOrient;
		  Vector worldPos;
		  worldPos.resize(3);
		  worldPos[0] = cmd->get(0).asDouble();
		  worldPos[1] = cmd->get(1).asDouble();
		  worldPos[2] = cmd->get(2).asDouble();
		  if(world_to_robot(worldPos,robotPos)){
#if 1
		    yarp::os::Time::delay(0.1);
		    if(trigger == 10) {
		      close_hand(arm_ctx,false);
		    }else if(trigger==100){
		      open_hand(arm_ctx,false);
		    }else{
		      yarp::dev::ICartesianControl* iArm=arm_ctx->iCartCtrl;
		      std::cout << "Moving to Pos: " << worldPos.toString(-1,1) << ",Orient: " << arm_ctx->init_orient.toString(-1,-1)<< std::endl;
#if 0
		      iArm->goToPoseSync(robotPos,arm_ctx->init_orient);
#else

#if SPOOF
		      set_cube_position(worldPos);
		      yarp::os::Time::delay(1);
#else
		      iArm->goToPoseSync(robotPos,arm_ctx->init_orient);
		      //iArm->goToPosition(robotPos);
		      if(!iArm->waitMotionDone()){
			std::cout << "Wait motion done failed!" << std::endl;
		      }
		      Vector curPos,curOrient,curWpos;
		      iArm->getPose(curPos,curOrient);
		      robot_to_world(curPos,curWpos);
		      std::cout << "Current Pos : " << curWpos.toString(-1,1) << " Current Orient : " << curOrient.toString(-1,1) << std::endl;
#endif

#endif  
		      arm_ctx->curr_world_position = worldPos;
		    }
#else
		    yarp::sig::Vector tmpQ, tmpX, tmpO;
		    tmpQ.resize(10);
		    tmpX.resize(3);
		    tmpO.resize(4);
		    yarp::sig::Vector postn = robotPos;
		    yarp::sig::Vector orntn = arm_ctx->init_orient;
		    Vector curPose;
		    curPose.resize(10);
		    for(int i=0;i<3;i++){
		      curPose[i]=partCtxMap["torso"]->current_pose[i];
		    }
		    for(int i=3;i<10;i++){
		      curPose[i]=arm_ctx->current_pose[i-3];
		    }
		    if(! arm_ctx->iCartCtrl->askForPosition(curPose,postn,tmpX,tmpO,tmpQ)){
			std::cout << " Inversion failed!!!\n ";
		    }
		    else{
		      Vector out;
		      bool grasp = false;
		      if(trigger == 10) grasp = true;
		      if(grasp){
			for(int i=8;i<16;i++){
			  tmpQ[i]=arm_ctx->close_pose[i-8];
			}
		      }else{
			for(int i=8;i<16;i++){
			  tmpQ[i]=arm_ctx->open_pose[i-8];
			}
		      }
		      move_joints_pos(arm_ctx->iPosCtrl,partCtxMap["torso"]->iPosCtrl,tmpQ,grasp,out);
		      for(int i=0;i<3;i++){
			partCtxMap["torso"]->current_pose[i]=curPose[i];
		      }
		      for(int i=3;i<10;i++){
			arm_ctx->current_pose[i-3]=curPose[i];
		      }
		    }
#endif
		  }
		}
		arm_ctx->status = MOVING;
		arm_ctx->action = REACHED;
	        actionTime = yarp::os::Time::now();
	      }
	    }
	  }
	}

      Bottle &portStatus = armStatusPort.prepare();
      portStatus.clear();
      if(arm_ctx->status == REACHED){
	arm_ctx->status = IDLE;
	portStatus.addDouble(1.0);
	//printf("Reached\n");
      }else if(arm_ctx->status == IDLE){
	portStatus.addDouble(10.0);
      }else if(arm_ctx->status == MOVING){
	portStatus.addDouble(100.0);
      }
      portStatus.addDouble(arm_ctx->curr_world_position[0]);
      portStatus.addDouble(arm_ctx->curr_world_position[1]);
      portStatus.addDouble(arm_ctx->curr_world_position[2]);
      double grasp=0;
      if(arm_ctx->graspStatus == GRASPED){
	grasp = 1;
      }
      portStatus.addDouble(grasp);
      armStatusPort.writeStrict();
     }
   }
#endif
}

void ArmControl::loop2()
{  
   std::map<std::string,boost::shared_ptr<PartContext> >::iterator it;
   it=partCtxMap.find(Config::instance()->root["armcontrol"]["arm"].asString());
   if(it!=partCtxMap.end()){
     boost::shared_ptr<ArmContext> arm_ctx = boost::dynamic_pointer_cast<ArmContext>(it->second);
     if(arm_ctx!=NULL){
        Bottle* cmd = armCmdPort.read(false);
	yarp::dev::ICartesianControl* iArm=arm_ctx->iCartCtrl;
	if(arm_ctx->status==MOVING){
	if(arm_ctx->graspStatus == GRASPING || arm_ctx->graspStatus==RELEASING){
	  std::cout << "Grasp Check! " << std::endl;
	  bool done;
	  if(arm_ctx->iPosCtrl->checkMotionDone(&done)){
	    if(done){
	      if(arm_ctx->graspStatus==GRASPING){
		std::cout << "Grasp Complete! " << std::endl;
		arm_ctx->graspStatus= GRASPED;
	      }
	      else if(arm_ctx->graspStatus==RELEASING){
		std::cout << "Open Complete! " << std::endl;
		arm_ctx->graspStatus= RELEASED;
	      }
	    }
	  }
	}else{
	  arm_ctx->status = arm_ctx->action;
	}
	}
	else{
	  if(cmd){
	    if(cmd->size() == 20){
	      //std::cout << "Received something: " << cmd->toString() << std::endl;
	      double trigger = cmd->get(19).asDouble();
	      if(trigger > 0){
		for(size_t p=0;p<cmd->size()-1;p++){
		  m_joints[p]=cmd->get(p).asDouble();
		}
		yarp::sig::Vector torso;torso.resize(3);
		yarp::sig::Vector arm;arm.resize(16);
		for(size_t p=0;p<3;p++){
		  torso[p]=cmd->get(p).asDouble();
		}
		for(size_t p=3;p<cmd->size()-1;p++){
		  arm[p-3]=cmd->get(p).asDouble();
		}
		move_joints(partCtxMap["torso"]->iPosCtrl,torso);
		move_joints(arm_ctx->iPosCtrl,arm);
		//arm_ctx->curr_world_position = worldPos;
		arm_ctx->status = MOVING;
		arm_ctx->action = REACHED;
	        actionTime = yarp::os::Time::now();
	      }
	    }
	  }
	}

      Bottle &portStatus = armStatusPort.prepare();
      portStatus.clear();
      if(arm_ctx->status == REACHED){
	arm_ctx->status = IDLE;
	portStatus.addDouble(1.0);
	//printf("Reached\n");
      }else if(arm_ctx->status == IDLE){
	portStatus.addDouble(10.0);
	//printf("Idle\n");
      }else if(arm_ctx->status == MOVING){
	portStatus.addDouble(100.0);
	//printf("Moving\n");
      }
      for(size_t p=0;p<m_joints.length();p++){
	portStatus.addDouble(m_joints[p]);
      }
      //cout << portStatus.toString() << std::endl;
      armStatusPort.writeStrict();
     }
   }
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
	config &= Utils::valueToVector(right_arm["open_hand"],right_ctx->open_pose);
	config &= Utils::valueToVector(right_arm["close_hand"],right_ctx->close_pose);
	
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
	if(!left_arm["init_pos"].isNull()){
	  config &= Utils::valueToVector(left_arm["init_pos"],left_ctx->init_world_position);
	}
	if(!left_arm["joint_speed"].isNull()){
	  left_ctx->joint_speed=left_arm["joint_speed"].asInt();
	}
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
    
    Time::turboBoost();
    
#if SPOOF
   make_dummy_cube();
   std::map<std::string,boost::shared_ptr<PartContext> >::iterator it;
   it=partCtxMap.find(Config::instance()->root["armcontrol"]["arm"].asString());
   if(it!=partCtxMap.end()){
      boost::shared_ptr<ArmContext> arm_ctx = boost::dynamic_pointer_cast<ArmContext>(it->second);
      if(arm_ctx!=NULL){
	Vector worldPos;worldPos.resize(3);
	worldPos[0]=-0.1;worldPos[1]=0.60;worldPos[2]=0.35;
	arm_ctx->curr_world_position = worldPos;
	arm_ctx->graspStatus=RELEASED;
      }
   }
#else
    initialize_robot();
    
#if 0
    // Move the robot to fixed initial position after the configuration is done.
    std::map<std::string,boost::shared_ptr<PartContext> >::iterator it;
    it=partCtxMap.find(Config::instance()->root["armcontrol"]["arm"].asString());
    if(it!=partCtxMap.end()){
      boost::shared_ptr<ArmContext> arm_ctx = boost::dynamic_pointer_cast<ArmContext>(it->second);
      if(arm_ctx!=NULL){
	if(arm_ctx->init_world_position.size()>0){
	  std::cout << "Moving " << partName <<  " to initial position " << arm_ctx->init_world_position.toString(-1,1) << std::endl;
	  Vector robotPos;
	  Vector pos;
	  Vector orient;
	  arm_ctx->iCartCtrl->getPose(pos,orient);
	  world_to_robot(arm_ctx->init_world_position,robotPos);
	  arm_ctx->iCartCtrl->goToPose(robotPos,orient);
	  if(!arm_ctx->iCartCtrl->waitMotionDone()){
	    std::cout << "Wait motion done failed!" << std::endl;
	  }
	  arm_ctx->iCartCtrl->getPose(pos,orient);
	  Vector worldPos;
	  robot_to_world(pos,worldPos);
	  arm_ctx->curr_world_position = arm_ctx->init_world_position;
	  std::cout << "Current Pos: (" << worldPos.toString(-1,1) <<  ") Orient: " << orient.toString(-1,1) << std::endl;
	}
      }
    }
#endif

#endif
    
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

#if 0
  // Dont bend more than 15 degrees
  double min, max;
  iArm->getLimits(0,&min,&max);
  iArm->setLimits(0,min,15);

  // Dont turn more than 15
  iArm->setLimits(2,-15,15);
  iArm->setLimits(1,-15,15);
#endif


  //Set joint reference speeds
  int axes;
  if(ctx->iPosCtrl->getAxes(&axes)){
    Vector speeds;
    speeds.resize(axes);
    for(int i=0;i<axes;i++){
      speeds[i]=ctx->joint_speed;
    }
    ctx->iPosCtrl->setRefSpeeds(speeds.data());
  }
  
  // print out some info about the controller
  Bottle info;
  iArm->getInfo(info);
  //iArm->getPose();
  std::cout << "info = " << info.toString() << "\n";
  std::cout << "DOFs = " << curDof.toString() << " and new: "
      << newDof.toString() << " \n";
  iArm->setTrackingMode(false);
  iArm->setPosePriority("orientation");
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
	    Vector robotPos;
	    arm_ctx->graspStatus=RELEASED;
	    arm_ctx->iCartCtrl->getPose(arm_ctx->init_position,arm_ctx->init_orient);
	    robotPos = arm_ctx->init_position;
	    Vector worldPos;
	    robot_to_world(robotPos,worldPos);
	    std::cout << "Initial Position: " << worldPos.toString() << std::endl;
	    std::cout << "Initial Orientation: " << arm_ctx->init_orient.toString() << std::endl;

// 	    for(size_t i=8;i<16;i++){
// 	      ctx->current_pose[i]=arm_ctx->open_pose[i-8];
// 	    }
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

bool ArmControl::close_hand(boost::shared_ptr<ArmContext>& ctx,bool bSync)
{
  bool bret = false;
  if(ctx!=NULL && ctx->iPosCtrl!=NULL && ctx->configured){
    if(ctx->graspStatus == RELEASED){
      std::cout << "Closing Hand: "  << std::endl;
      ctx->graspStatus = GRASPING;
      yarp::sig::Vector close_pose = ctx->close_pose;
      std::cout << close_pose.toString() << std::endl;
      for(size_t i=0; (i<close_pose.size()) && (i<8); i++)
      {        
	  ctx->iPosCtrl->setRefSpeed(i+8, 100);
	  ctx->iPosCtrl->positionMove(i+8, close_pose[i]);
      }
      if(bSync){
	bool done=false;
	while (!done) {
	  ctx->iPosCtrl->checkMotionDone(&done);
	  yarp::os::Time::delay(0.1);   // or any suitable delay
	}
	ctx->graspStatus = GRASPED;
        std::cout << "Closed Hand: " << std::endl;
      }
      else{
	yarp::os::Time::delay(1);
	std::cout << "Closed Hand: " << std::endl;
      }
      ctx->graspStatus = GRASPED;
      bret = true;
    }
  }
  return bret;
}


bool ArmControl::open_hand(boost::shared_ptr<ArmContext>& ctx,bool bSync)
{
  bool bret = false;
  if(ctx!=NULL && ctx->iPosCtrl!=NULL && ctx->configured){
    if(ctx->graspStatus == GRASPED){
      std::cout << "Opening Hand: " << std::endl;
      ctx->graspStatus = RELEASING;
      yarp::sig::Vector open_pose = ctx->open_pose;
      for(size_t i=1; (i<open_pose.size()) && (i<8); i++)
      {        
	  ctx->iPosCtrl->setRefSpeed(i+8, 100);
	  ctx->iPosCtrl->positionMove(i+8, open_pose[i]);
      }        
      yarp::os::Time::delay(1.0);
      ctx->iPosCtrl->setRefSpeed(8, 100);
      ctx->iPosCtrl->positionMove(8, open_pose[0]);
      if(bSync){
	bool done=false;
	while (!done) {
	  ctx->iPosCtrl->checkMotionDone(&done);
	  yarp::os::Time::delay(0.01);   // or any suitable delay
	}
	ctx->graspStatus = RELEASED;
	std::cout << "Opened Hand: " << std::endl;
      }else{
	yarp::os::Time::delay(1);
	std::cout << "Opened Hand: " << std::endl;
      }
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
      yarp::sig::Vector speeds;
      for(size_t p=0;p<qd.size();p++){
	speeds.push_back(30);
      }
      posCtrl->setRefSpeeds(speeds.data());
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
    std::cout << "Moved to : " << qd.toString() << std::endl;
    return done;
}

void ArmControl::move_joints_pos(IPositionControl* armPosCtrl,IPositionControl* torsoPosCtrl, yarp::sig::Vector &qd,bool grasp,yarp::sig::Vector& out)
{
    bool done = false;
    yarp::sig::Vector torsoPos;
    torsoPos.resize(3);
    for(int i=0 ; i < 3; i++)
    {
	torsoPos[i] = qd[i];
    }
    torsoPosCtrl->positionMove(torsoPos.data());
    do
    {
	torsoPosCtrl->checkMotionDone(&done);
	yarp::os::Time::delay(.01);
    }while(!done);

    yarp::sig::Vector armPos;
    armPos.resize(16);
    
    for(int i=0; i < 8; i++)
    {
	armPos[i] = qd[i+3];
    }
    
    for(Json::ArrayIndex i=8; i<16; i++)
    {
      if(!grasp){
	armPos[i] = Config::instance()->root["robot"]["parts"]["left_arm"]["open_hand"][i-8].asDouble();
      }else{
	armPos[i] = Config::instance()->root["robot"]["parts"]["left_arm"]["close_hand"][i-8].asDouble();
      }
    }
    
    armPosCtrl->positionMove(armPos.data());
    
    do
    {
	armPosCtrl->checkMotionDone(&done);
	yarp::os::Time::delay(.01);
    }while(!done);
    
    std::cout << "Move to pos: "
	      << torsoPos.toString() << " and arm: "
	      << armPos.toString() << "\n";
	      
    out=armPos;
	
    if(!grasp)
      std::cout << "Opened Hand!" << std::endl;		
    else
      std::cout << "Closed Hand!" << std::endl;		
}
