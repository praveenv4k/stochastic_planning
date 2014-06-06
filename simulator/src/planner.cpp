
#include "planner.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

#include <math.h>

void Planner::loop()
{
   Bottle* cmd = plannerCmdPort.read(false);
   if(cmd)
   {
     bool send = false;
     double command = cmd->get(0).asDouble();
     if(command == 1){
       printf("Received response %lf:\n",command);
       send = false;
     }else if(command == 10){
       send = true;
	//printf("Received response %lf:\n",command);
     }else if(command == 100){
	printf("Arm is Moving : %lf\n",command);
     }

     if(send){
      t=Time::now();
      double x=-0.3;
      double y=-0.1+0.1*cos(2.0*M_PI*0.1*(t-t0));
      double z=+0.1+0.1*sin(2.0*M_PI*0.1*(t-t0));  
      double trigger=1;
      Bottle &status = plannerStatusPort.prepare();
      status.clear();
      status.addDouble(x);
      status.addDouble(y);
      status.addDouble(z);
      status.addDouble(trigger);
      plannerStatusPort.write();  
      printf("Move request sent\n");
     }
   }
}

bool Planner::open(yarp::os::ResourceFinder &rf)
{   
    bool ret=true;   
    ret = plannerCmdPort.open("/planner/cmd/in");
    ret &= plannerStatusPort.open("/planner/status/out");     
    
//     if(! Network::connect("/graspObject/status/out","/planner/cmd/in"))
//     {
//         std::cout <<": unable to connect to grasp object output port.\n";
//         return false;  // unable to open; let RFModule know so that it won't run
//     }
//     
//     if(! Network::connect("/planner/status/out","/graspObject/cmd/in"))
//     {
//         std::cout <<": unable to connect to grasp object input port.\n";
//         return false;  // unable to open; let RFModule know so that it won't run
//     }
    
//     if(! Network::connect("/armcontrol/status/out","/planner/cmd/in"))
//     {
//         std::cout <<": unable to connect to arm control output port.\n";
//         return false;  // unable to open; let RFModule know so that it won't run
//     }
//     
//     if(! Network::connect("/planner/status/out","/armcontrol/cmd/in"))
//     {
//         std::cout <<": unable to connect to arm control input port.\n";
//         return false;  // unable to open; let RFModule know so that it won't run
//     }
    
//     Port port; 
//     Bottle reply;
//     port.open("/move_ball"); 
//     
//     if(! Network::connect("/move_ball","/icubSim/world"))
//     {
//         std::cout <<": unable to connect ball object to world.\n";
//         return false;  // unable to open; let RFModule know so that it won't run
//     }
//     Bottle del_all("world del all"); 
//     port.write(del_all,reply); 
//     Bottle create_obj("world mk ssph 0.02 0.0 0.8 0.4 0 1 0");
//     port.write(create_obj,reply);
    
    t=t1=t0 = Time::now();
    return ret;
}

bool Planner::close()
{
    //iHand->stop();
    plannerCmdPort.close();
    plannerStatusPort.close();
    return true;
}

bool Planner::interrupt()
{
    plannerCmdPort.interrupt();
    return true;
}


