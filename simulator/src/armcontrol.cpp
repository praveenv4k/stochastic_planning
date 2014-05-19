
#include "armcontrol.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

#include <math.h>


void ArmControl::loop()
{
   Bottle* cmd = armCmdPort.read(false);
   if(cmd && (status != MOVING))
   {
     if(cmd->size() == 4){
       if(cmd->get(4).asDouble() == 1)
       {
	 std::cout << "Received new position" << std::endl;
            Vector handPos,handOrient;
	    handPos[0] = cmd->get(0).asDouble();
	    handPos[1] = cmd->get(1).asDouble();
	    handPos[2] = cmd->get(2).asDouble();
            iArm->getPose(handPos, handOrient);

	    yarp::os::Time::delay(1.5);
            action = REACHED;
       }
       status = MOVING;
       actionTime = yarp::os::Time::now();
     }
   }


    bool motionDone;   
    if(iHand->checkMotionDone(&motionDone))
    {
	if(motionDone && 
	    (yarp::os::Time::now() - actionTime) >= 3.0 )
	    status = action;
    }        

    if(status == REACHED || first)
    {
      status = IDLE;
      Bottle &portStatus = armStatusPort.prepare();
      portStatus.clear();
      portStatus.addDouble(1.0);
      armStatusPort.write();  
      printf("Reached\n");
      first = false;
    }
}

// bool ArmControl::open(yarp::os::ResourceFinder &rf)
bool ArmControl::open()
{   
    string ctrlName;
    string robotName;
    string remoteName;
    string localName;

    //Time::turboBoost();

    // get params from the RF    
    partName = "right";
 
    // opening hand controller
    robotName = "icubSim";
    remoteName = string("/")+robotName+string("/")+partName+string("_arm");
    localName = string("/armcontrol/")+partName+string("_hand");
 
    // open the client
    Property option("(device remote_controlboard)");
    option.put("remote",remoteName.c_str());
    option.put("local",localName.c_str());
    if (!driver.open(option)){
      std::cout << "Remote control board cannot be opened"<<std::endl;
        return false;
    }
    // open the view
    driver.view(iHand);

    // opening arm controller
    remoteName=string("/")+robotName+"/cartesianController/"+partName+string("_arm");
    localName=string("/armcontrol/")+partName+string("_arm");
    // open the client
    Property armOption("(device cartesiancontrollerclient)");
    armOption.put("remote",remoteName.c_str());
    armOption.put("local",localName.c_str());
    if (!armDriver.open(armOption))
        return false;

    // open the view
    armDriver.view(iArm);
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


    action = status = IDLE;


    bool ret=true;   
    ret = armCmdPort.open("/armcontrol/cmd/in");
    ret &= armStatusPort.open("/armcontrol/status/out");   
    return ret;
}

bool ArmControl::close()
{
    driver.close();
    armDriver.close();
    armCmdPort.close();
    armStatusPort.close();
    return true;
}

bool ArmControl::interrupt()
{
    armCmdPort.interrupt();
    return true;
}


