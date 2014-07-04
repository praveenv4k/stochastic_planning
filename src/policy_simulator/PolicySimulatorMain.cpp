#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>

#include <stdio.h>

#include "PolicySimulator.h"
#include <json/json.h>
#include "Config.h"
#include "ElapsedTime.h"

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

class PolicySimulatorModule: public RFModule
{
   PolicySimulator* planner;
   Json::Value root;
public:

    PolicySimulatorModule()
    {
      planner = new PolicySimulator();
    }

    ~PolicySimulatorModule()
    {
      delete planner;        
    }
    
    bool checkModelCheckerFileGen(){
      bool ret=false;
      root = Config::instance()->root;
      std::string policy_folder = root["simulator"]["policy_path"].asString();
      
      if(!policy_folder.empty()){
	
	std::string actionFile = policy_folder+"action.txt";
	std::string statesFile = policy_folder+"states.txt";
	std::string policyMap = policy_folder+"policymap.txt";
	std::string collision = policy_folder+"collision.txt"; 
	std::string modelcheck = policy_folder+"modelcheck";
	std::string sinkFile = policy_folder+"sinkstates.txt";
	std::string dtmcFile = policy_folder+"dtmc";
	
	{
	  ElapsedTime elapsed("Reading Actions Map");
	  planner->read_actions(actionFile);
	}
	{
	  ElapsedTime elapsed("Reading States Map");
	  planner->read_states(statesFile);
	}
	{
	  ElapsedTime elapsed("Reading Policy map");
	  planner->read_policy_map(policyMap);
	}
	{
	  ElapsedTime elapsed("Read Collision Map");
	  planner->read_collision(collision);
	}
	{
	  ElapsedTime elapsed("Read Sink states File");
	  planner->read_sink_states(sinkFile);
	}
	{
	  ElapsedTime elapsed("Generate model checker file");
	  
	  double mean=0,sigma=1;
	  std::string modelcheckTemp = modelcheck+"01.nm";
	  planner->generateModelCheckerFile(modelcheckTemp,mean,sigma);
	  
	  mean=0,sigma=3;
	  modelcheckTemp = modelcheck+"03.nm";
	  planner->generateModelCheckerFile(modelcheckTemp,mean,sigma);
	  
	  mean=1,sigma=1;
	  modelcheckTemp = modelcheck+"11.nm";
	  planner->generateModelCheckerFile(modelcheckTemp,mean,sigma);
	  
	  mean=2,sigma=1;
	  modelcheckTemp = modelcheck+"21.nm";
	  planner->generateModelCheckerFile(modelcheckTemp,mean,sigma);
	}
	{
	  ElapsedTime elapsed("Generate DTMC file");
	  //planner->generateDtmcFile(dtmcFile);
	  
	  double mean=0,sigma=1;
	  std::string dtmcFileTemp = dtmcFile+"01.dtmc";
	  planner->generateDtmcFile(dtmcFileTemp,mean,sigma);
	  
	  mean=0,sigma=3;
	  dtmcFileTemp = dtmcFile+"03.dtmc";
	  planner->generateDtmcFile(dtmcFileTemp,mean,sigma);
	  
	  mean=1,sigma=1;
	  dtmcFileTemp = dtmcFile+"11.dtmc";
	  planner->generateDtmcFile(dtmcFileTemp,mean,sigma);
	  
	  mean=2,sigma=1;
	  dtmcFileTemp = dtmcFile+"21.dtmc";
	  planner->generateDtmcFile(dtmcFileTemp,mean,sigma);
	}
	return true;
      }
      else{
	return false;
      }
    }

    bool configure(ResourceFinder &rf)
    {
      bool ret=false;
      root = Config::instance()->root;
      std::string policy_folder = root["simulator"]["policy_path"].asString();
      
      if(!policy_folder.empty()){
	
	std::string actionFile = policy_folder+"action.txt";
	std::string statesFile = policy_folder+"states.txt";
	std::string domainFile = policy_folder+"domain.pomdpx";
	std::string policyFile = policy_folder+"domain.policy";
	std::string policyMap = policy_folder+"policymap.txt";
	{
	  ElapsedTime elapsed("Reading Actions Map");
	  planner->read_actions(actionFile);
	}
	{
	  ElapsedTime elapsed("Reading States Map");
	  planner->read_states(statesFile);
	}
#if USE_LOOP2
	{
	  ElapsedTime elapsed("Reading Policy map");
	  planner->read_policy_map(policyMap);
	}
#else
	{
	  ElapsedTime elapsed("Reading Policy File");
	  planner->read_policy(domainFile,policyFile);
	}
	{
	  ElapsedTime elapsed("Policy initialization");
	  planner->initialize_plan();
	}
#endif
      }
      else{
	return false;
      }
      
      if(planner->open(rf)){
	Network yarp;
	if(! Network::connect("/armcontrol/status/out","/planner/cmd/in")){
	  std::cout <<": unable to connect to arm control output port.\n";
	}
	else{
         if(! Network::connect("/planner/status/out","/armcontrol/cmd/in")){
          std::cout <<": unable to connect to arm control input port.\n";
         }else{
	   if(! Network::connect("/planobj/status/out","/object/cmd/in")){
            std::cout <<": unable to connect to object control input port.\n";
           }else{
	     if(! Network::connect("/object/status/out","/planobj/cmd/in")){
	      std::cout <<": unable to connect to planner input port.\n";
	     }else{
	      ret=true;
	     }
	   }
	 }   
       }
      }
      return ret;
    }

    double getPeriod( )
    {
        return 0.1;        
    }
    
    bool updateModule()
    { 
#if USE_LOOP2
	planner->loop2();
#else
        planner->loop();
#endif
        return true; 
    }

    bool interruptModule()
    {
        fprintf(stderr, "Interrupting\n");
        planner->interrupt();
        return true;
    }

    bool close()
    {
        fprintf(stderr, "Calling close\n");
        planner->close();
        return true;
    }
};

int main(int argc, char *argv[])
{   
#if 0
  // we need to initialize the drivers list 
    YARP_REGISTER_DEVICES(icubmod)

    Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf(stdout,"Error: yarp server does not seem available\n");
        return -1;
    }
    PolicySimulatorModule module;
    ResourceFinder rf;
    return module.runModule(rf);
#else
    PolicySimulatorModule module;
    return module.checkModelCheckerFileGen();
#endif
}