
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
       //printf("Received response %lf:\n",command);
       send = false;
//        posQueue.pop();
     }else if(command == 10){
       if(sent==true){
	 posQueue.pop();
	 sent = false;
       }
        send = true;
	//printf("Arm is Idle response %lf:\n",command);
     }else if(command == 100){
	//printf("Arm is Moving : %lf\n",command);
     }

     if(send){
#if 1
       if(posQueue.size()>0){
	 Vector& v = posQueue.front();
	 double x=v[0];
	 double y=v[1];
	 double z=v[2];  
	 double trigger=v[3];
	 
	 Bottle &status = plannerStatusPort.prepare();
	 status.clear();
	 status.addDouble(x);
	 status.addDouble(y);
	 status.addDouble(z);
	 status.addDouble(trigger);
	 plannerStatusPort.write();  
	 std::cout << "Target : " << v.toString() << std::endl;
	 printf("Move request sent\n");
	 sent=true;
       }
#else
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
#endif      
      //printf("Move request sent\n");
     }
   }
}

bool Planner::open(yarp::os::ResourceFinder &rf)
{   
    bool ret=true;   
    ret = plannerCmdPort.open("/planner/cmd/in");
    ret &= plannerStatusPort.open("/planner/status/out");     
        
    Vector v1;
    v1.resize(4);
    v1[0]= 0.0;
    v1[1]= 0.6;
    v1[2]=0.25;
    v1[3]=1;
    posQueue.push(v1);
    v1[0]= 0.0;
    v1[1]= 0.6;
    v1[2]=0.25;
    v1[3]=10;
    posQueue.push(v1);
    
    printf("Targets count: %d\n",posQueue.size());
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


