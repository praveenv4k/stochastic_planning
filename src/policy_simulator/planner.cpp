
#include "planner.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

#include <math.h>
#include <fstream>
#include "Config.h"

#define DEBUG_TRACE_ON 1

void Planner::loop()
{
  if(first){
    first=false;
    Bottle &status = planobjStsPort.prepare();
    status.clear();
    status.addDouble(1);
    planobjStsPort.writeStrict();
    
  }else{
    if(reached&&!execStop){
      Bottle &status = planobjStsPort.prepare();
      status.clear();
      status.addDouble(0);
      planobjStsPort.writeStrict();
      execStop = true;
      m_ElaspedTime->pause();
      //m_ElaspedTime->printElasped();
      std::cout << "Target Reached : Exec stop" << std::endl;
    }
  }
   Bottle* objCmd = planobjCmdPort.read(false);
   if(objCmd){
     objPosition[0] = objCmd->get(0).asDouble();
     objPosition[1] = objCmd->get(1).asDouble();
     objPosition[2] = objCmd->get(2).asDouble();
     noisyObjPosition[0] = objCmd->get(3).asDouble()*100;
     noisyObjPosition[1] = objCmd->get(4).asDouble()*100;
     noisyObjPosition[2] = objCmd->get(5).asDouble()*100;
   }
   Bottle* cmd = plannerCmdPort.read(false);
   if(cmd)
   {
     bool send = false;
     double command = cmd->get(0).asDouble();
     for(size_t i=0;i<4;i++){
      robotPosition[i] = cmd->get(i+1).asDouble();
     }
     for(int i=0;i<4;i++){
       augState[i]=round(robotPosition[i]*100);
     }
     for(int i=4;i<7;i++){
       augState[i]=round(objPosition[i-4]*100);
     }
     if(command == 1){
       if(sent == true){
	 posQueue.pop();
	 sent = false;
	 
#ifdef VECTOR_PTR_MAP
	 VectorPtr ptr(new yarp::sig::Vector());
	 for(size_t i=0;i<augState.size();i++){
	   ptr->push_back(augState[i]);
	 }
	 std::cout << "Trying to retrieve the augmented state: " << ptr->toString(-1,1) << std::endl;
	 VectorIndexMap::iterator found = m_VectorMap->find(ptr);
	 if(found!=m_VectorMap->end()){
	   std::cout << "Found the augmented state!" << std::endl;
	  int state = m_VectorMap->operator[](ptr);
	  std::cout << "Index: " << state << "; State: ("<<ptr->toString(-1,1) << ")" << std::endl;
	 }
	  std::cout << "Ready to take next action!" << std::endl;
        }
#else
	 std::vector<double> val;
	 for(size_t i=0;i<augState.size();i++){
	   val.push_back(augState[i]);
	   std::cout << val[i] << " ";
	 }
	 
	 bool reach = has_reached(augState,graspThreshold);
	 if(reach){
	   reached = reach;
	 }else{
	  StateIndexMap::iterator found = m_StateIndexMap.find(val);
	  if(found!=m_StateIndexMap.end()){
	      int state = found->second;
	      std::cout << "Index: " << state << std::endl;
	      
	      prevState = currBelSt->sval;
	      currBelSt->sval=state;
	      VectorPtr prev_v=m_States->operator[](currBelSt->sval);
	      cout << " Current State : " << currBelSt->sval;
	      action = runFor(1,NULL,reward,expReward);
	      cout << " Best Action : " << action <<  " Next State: " << currBelSt->sval << std::endl;
	      if(action!=-1 && !reached){// && prevState!=currBelSt->sval){
		VectorPtr v=m_States->operator[](currBelSt->sval);
		Vector v1;
		v1.resize(4);
		v1[0]= v->operator[](0)/100;
		v1[1]= v->operator[](1)/100;
		v1[2]= v->operator[](2)/100;
		v1[3]= 1;
		if(!(fabs(v->operator[](0)-val[0])<1e-2)||
		   !(fabs(v->operator[](1)-val[1])<1e-2)||
		   !(fabs(v->operator[](2)-val[2])<1e-2)){
		    std::cout << "New position pushed to queue!" << std::endl;
		    posQueue.push(v1);
		    send = true;
		}
		
		reached = has_reached(*v,graspThreshold);
	      }else{
		// Stop the ball
		Bottle &status = planobjStsPort.prepare();
		status.clear();
		status.addDouble(0);
		planobjStsPort.writeStrict();
	      }
	  }else{
	    std::cout << "Augmented state (" << val << ") not found!" << std::endl;
	  }
	}
     }
#endif
       //send = true;
     }else if(command == 10){
       if(sent==true){
        }else{
	}
	if(!reached){
	  reached = has_reached(augState,graspThreshold);
	}
     }else if(command == 100){
     }

     if(send || firstSend){
       if(firstSend) firstSend=false;
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
	 plannerStatusPort.writeStrict();  
	 std::cout << "Target : " << v.toString() << std::endl;
	 printf("Move request sent\n");
	 sent=true;
       }
     }
   }
}

bool Planner::has_reached(yarp::sig::Vector& augState,double graspThreshold){
  bool ret=false;
  if(augState.size()==7){
    Vector robot,object;
    robot.resize(3);object.resize(3);
#if 0
    for(int i=0;i<3;i++){
      robot[i]=augState[i];
      object[i]=augState[i+4];
    }
#else
    for(int i=0;i<3;i++){
      robot[i]=augState[i];
      object[i]=noisyObjPosition[i];
    }
#endif
    object[1]+=radius;
    dist = Planner::computeL2norm<Vector>(robot,object);
    cout << "Distance Threshold : (" << robot.toString(-1,1) << "),(" <<object.toString(-1,1) << ") :" << dist << std::endl;
    if(dist < graspThreshold /*&& augState[3]>0*/){
      cout << "Reached close to object!" << std::endl;
      ret=true;
    }
  }
  return ret;
}

bool Planner::open(yarp::os::ResourceFinder &rf)
{   
    bool ret=true;   
    ret = plannerCmdPort.open("/planner/cmd/in");
    ret &= plannerStatusPort.open("/planner/status/out"); 
    ret &= planobjCmdPort.open("/planobj/cmd/in");
    ret &= planobjStsPort.open("/planobj/status/out"); 
    {
      VectorPtr v=m_States->operator[](currBelSt->sval);
      Vector v1;
      v1.resize(4);
      v1[0]= v->operator[](0)/100;
      v1[1]= v->operator[](1)/100;
      v1[2]= v->operator[](2)/100;
      v1[3]= 1;
      posQueue.push(v1);
    }
#if 0
    int action=-1;
    double reward=0,expReward=0;
    double dist = 100000;
    double graspThreshold = 1.6;
    bool reached=false;
    while(!reached){
      int prevState = currBelSt->sval;
      VectorPtr prev_v=m_States->operator[](currBelSt->sval);
      cout << " Current State : " << currBelSt->sval;
      action = runFor(1,NULL,reward,expReward);
      cout << " Best Action : " << action <<  " Next State: " << currBelSt->sval << std::endl;
      if(action!=-1 && prevState!=currBelSt->sval){
	VectorPtr v=m_States->operator[](currBelSt->sval);
	Vector v1;
	v1.resize(4);
	v1[0]= v->operator[](0)/100;
	v1[1]= v->operator[](1)/100;
	v1[2]= v->operator[](2)/100;
	v1[3]= 1;
	posQueue.push(v1);
	
	Vector robot,object;
	robot.resize(3);object.resize(3);
	for(int i=0;i<3;i++){
	  robot[i]=v->operator[](i);
	  object[i]=v->operator[](i+4);
	}
	object[1]=object[1]+3;
	dist = Planner::computeL2norm<Vector>(robot,object);
	cout << "Distance Threshold : " << dist << std::endl;
	if(dist < graspThreshold && v->operator[](3)>0){
	  cout << "Reached close to object!" << std::endl;
	  reached=true;
	}
      }else{
	break;
      }
    }
    cout << " Reward: " << reward << " Expected Reward: " << expReward << std::endl;

    printf("Targets count: %d\n",posQueue.size());
#endif
    t=t1=t0 = Time::now();
    //m_ElaspedTime->start();
    return ret;
}

bool Planner::close()
{
    plannerCmdPort.close();
    plannerStatusPort.close();
    return true;
}

bool Planner::interrupt()
{
    plannerCmdPort.interrupt();
    return true;
}

bool Planner::read_states(std::string states_file){
  bool bret = false;
  if(!states_file.empty()){
    std::ifstream fs(states_file.c_str(),std::ifstream::in);
    string str;
    while(std::getline(fs,str)){
      std::vector<double> vec;
      if(!str.empty()){
	if(parse_state_action(str,vec)){
	  VectorPtr ptr(new yarp::sig::Vector());
	  for(size_t i=1;i<vec.size()-1;i++){
	    ptr->push_back(vec[i]);
	  }
	  m_States->insert(std::pair<int,VectorPtr>((int)vec[0],ptr));

#ifdef VECTOR_PTR_MAP
	  m_VectorMap->insert(std::pair<VectorPtr,int>(ptr,(int)vec[0]));
#else
	  std::vector<double> val;
	  for(size_t i=1;i<vec.size()-1;i++){
	    val.push_back(round(vec[i]));
	  }
	  m_StateIndexMap.insert(std::pair<std::vector<double>,int>(val,(int)vec[0]));
#endif
	}
      }
    }
    bret = true;
  }
  std::cout << "Number of states in States Map: " << m_States->size() <<std::endl;
#ifdef VECTOR_PTR_MAP
  std::cout << "Number of states in Vector Map: " << m_VectorMap->size() <<std::endl;
#else
  std::cout << "Number of states in State Index Map: " << m_StateIndexMap.size() <<std::endl;
#endif
  return bret;
}

bool Planner::read_actions(std::string actions_file){
  bool bret = false;
  if(!actions_file.empty()){
    std::ifstream fs(actions_file.c_str(),std::ifstream::in);
    string str;
    while(std::getline(fs,str)){
      std::vector<double> vec;
      if(!str.empty()){
	if(parse_state_action(str,vec)){
	  VectorPtr ptr(new yarp::sig::Vector());
	  for(size_t i=1;i<vec.size();i++){
	    ptr->push_back(vec[i]);
	  }
	  m_Actions->insert(std::pair<int,VectorPtr>((int)vec[0],ptr));
	}
      }
    }
    bret = true;
  }
  std::cout << "Number of Actions: " << m_Actions->size() <<std::endl;
  return bret;
}

bool Planner::read_policy(std::string domain_file, std::string policy_file){
  try
    {
      SolverParams* p = &GlobalResource::getInstance()->solverParams;

      p->policyFile=policy_file;
      p->problemName = domain_file;
      p->simLen = 1;
      p->simNum=100;

      //check validity of options
      if (p->policyFile == "" || p->simLen == -1 || p->simNum == -1) 
      {
	  return false;
      }

      cout << "\nLoading the model ..." << endl << "  ";
      problem = ParserSelector::loadProblem(p->problemName, *p);

      if(p->stateMapFile.length() > 0 )
      {
	  // generate unobserved state to variable value map
	  ofstream mapFile(p->stateMapFile.c_str());
	  for(int i = 0 ; i < problem->YStates->size() ; i ++)
	  {
	      mapFile << "State : " << i <<  endl;
	      map<string, string> obsState = problem->getFactoredUnobservedStatesSymbols(i);
	      for(map<string, string>::iterator iter = obsState.begin() ; iter != obsState.end() ; iter ++)
	      {
		  mapFile << iter->first << " : " << iter->second << endl ;
	      }
	  }
	  mapFile.close();
      }

      policy = new AlphaVectorPolicy(problem);

      cout << "\nLoading the policy ..." << endl;
      cout << "  input file   : " << p->policyFile << endl;
      bool policyRead = policy->readFromFile(p->policyFile);
      if(!policyRead)
      {
	  return 0;
      }

      cout << "\nSimulating ..." << endl;
      
      if(p->useLookahead)
      {
	  cout << "  action selection :  one-step look ahead" << endl;
      }
      else
      {
      }

      rewardCollector->setup(*p);

      srand(p->seed);
      
      this->policy = policy;
      this->problem = problem;
      this->solverParams = p;
    }
    catch(bad_alloc &e)
    {
        if(GlobalResource::getInstance()->solverParams.memoryLimit == 0)
        {
            cout << "Memory allocation failed. Exit." << endl;
        }
        else
        {
            cout << "Memory limit reached. Please try increase memory limit" << endl;
        }

    }
    catch(exception &e)
    {
        cout << "Exception: " << e.what() << endl ;
    }
    return 0;
}

bool Planner::initialize_plan(){
  engine->setup(problem,policy,solverParams);
  DEBUG_TRACE(cout << "runFor" << endl; );
  DEBUG_TRACE(cout << "iters " << iters << endl; );
  DEBUG_TRACE(cout << "startBeliefX" << endl; );
  DEBUG_TRACE(startBeliefX.write(cout) << endl;);
  
  // get starting actStateCompl, the complete state (X and Y values)
  if (problem->initialBeliefStval->sval == -1) // check if the initial starting state for X is fixed
  {
    // random starting state for X
    const SharedPointer<DenseVector>& startBeliefX = problem->initialBeliefX;
    //cout << startBeliefX->ToString() << std::endl;
#if 0
    actStateCompl->sval = chooseFromDistribution(*startBeliefX);
#else
    //actStateCompl->sval = 441; // Stationary 26 June ...
    //actStateCompl->sval = 770; // Stationary 27 June test1
    //actStateCompl->sval = 7850; // Straight Line 27 June test1 (7851 13 60 48 0 -11 53.39 35 27.5324)
    if(!Config::instance()->root["simulator"]["start_state"].isNull()){
      actStateCompl->sval = Config::instance()->root["simulator"]["start_state"].asInt();
    }else{
      const SharedPointer<DenseVector>& startBeliefX = problem->initialBeliefX;
      actStateCompl->sval = chooseFromDistribution(*startBeliefX);
    }
#endif
    copy(currBelX, *startBeliefX);
    cout << "Random initial state: " <<  actStateCompl->sval << endl;
  }
  else
  {
    cout << "Fixed initial state: " <<  problem->initialBeliefStval->sval << endl;
    // initial starting state for X is fixed
    actStateCompl->sval = problem->initialBeliefStval->sval;
  }

  // now choose a starting unobserved state for the actual system
  SharedPointer<SparseVector> startBeliefVec;
  if (problem->initialBeliefStval->bvec)
    startBeliefVec = problem->initialBeliefStval->bvec;
  else
    startBeliefVec = problem->initialBeliefYByX[actStateCompl->sval];
  
  currUnobsState = chooseFromDistribution(*startBeliefVec);
  belSize = startBeliefVec->size();

  actStateCompl->bvec->resize(belSize);
  actStateCompl->bvec->push_back(currUnobsState, 1.0);

  DEBUG_TRACE( cout << "actStateCompl sval " << actStateCompl->sval << endl; );
  DEBUG_TRACE( actStateCompl->bvec->write(cout) << endl; );

  currBelSt->sval = actStateCompl->sval;
  copy(*currBelSt->bvec, *startBeliefVec);

  DEBUG_TRACE( cout << "currBelSt sval " << currBelSt->sval << endl; );
  DEBUG_TRACE( currBelSt->bvec->write(cout) << endl; );
  
  return true;
}

int Planner::runFor(int iters, ofstream* streamOut, double& reward, double& expReward)
{ 
  //double mult=1;
  CPTimer lapTimer;
  int firstAction;
  double gamma = problem->getDiscount();
    for(int timeIter = 0; timeIter < iters; timeIter++)
    { 
	DEBUG_TRACE( cout << "timeIter " << timeIter << endl; );

	// get action according to policy and current belief and state
	//int currAction;
	//-----------------------------
	if (timeIter == 0)
	{
	    if(solverParams->useLookahead)
	    {
		if (currBelSt->sval == -1) // special case for first time step where X is a distribution
		    currAction = policy->getBestActionLookAhead(currBelSt->bvec, currBelX);
		else 
		    currAction = policy->getBestActionLookAhead(*currBelSt);
	    }
	    else
	    {
		if (currBelSt->sval == -1) // special case for first time step where X is a distribution
		    currAction = policy->getBestAction(currBelSt->bvec, currBelX);
		else 
		    currAction = policy->getBestAction(*currBelSt);
	    }
	}   
	else
	{

	    if(solverParams->useLookahead)
	    {
		    currAction = policy->getBestActionLookAhead(*currBelSt);
		    
	    }
	    else
	    {
		    currAction = policy->getBestAction(*currBelSt);
	    }
	}

	if(currAction < 0 )
	{
	    cout << "You are using a MDP Policy, please make sure you are using a MDP policy together with one-step look ahead option turned on" << endl;
	    return -1;
	}
	if (timeIter == 0)
	{
	    firstAction = currAction;
	}

	// this is the reward for the "actual state" system
	currReward =engine->getReward(*actStateCompl, currAction);

	DEBUG_TRACE( cout << "currAction " << currAction << endl; );
	DEBUG_TRACE( cout << "actStateCompl sval " << actStateCompl->sval << endl; );
	DEBUG_TRACE( actStateCompl->bvec->write(cout) << endl; );

	DEBUG_TRACE( cout << "currReward " << currReward << endl; );
	expReward += mult*currReward;
	mult *= gamma;
	reward += currReward;

	DEBUG_TRACE( cout << "expReward " << expReward << endl; );
	DEBUG_TRACE( cout << "reward " << reward << endl; );


	// actualActionUpdObs is belief of the fully observered state
	belief_vector actualActionUpdUnobs(belSize), actualActionUpdObs(problem->XStates->size()) ;
	engine->performActionObs(actualActionUpdObs, currAction, *actStateCompl);

	DEBUG_TRACE( cout << "actualActionUpdObs " << endl; );
	DEBUG_TRACE( actualActionUpdObs.write(cout) << endl; );

	// the actual next state for the observed variable
	actNewStateCompl->sval = (unsigned int) chooseFromDistribution(actualActionUpdObs, ((double)rand()/RAND_MAX));

	// now update actualActionUpdUnobs, which is the belief of unobserved states,
	// based on prev belif and curr observed state
	engine->performActionUnobs(actualActionUpdUnobs, currAction, *actStateCompl, actNewStateCompl->sval);
	
	DEBUG_TRACE( cout << "actualActionUpdUnobs " << endl; );
	DEBUG_TRACE( actualActionUpdUnobs.write(cout) << endl; );

	// the actual next state for the unobserved variable
	int newUnobsState = chooseFromDistribution(actualActionUpdUnobs, ((double)rand()/RAND_MAX));

	DEBUG_TRACE( cout << "newUnobsState "<< newUnobsState << endl; );

	actNewStateCompl->bvec->resize(belSize);
	actNewStateCompl->bvec->push_back(newUnobsState, 1.0);

	DEBUG_TRACE( cout << "actNewStateCompl sval "<< actNewStateCompl->sval << endl; );
	DEBUG_TRACE( actNewStateCompl->bvec->write(cout) << endl; );

	// get observations based on actual next states for observed and unobserved variable
	belief_vector obsPoss;
	engine->getPossibleObservations(obsPoss, currAction, *actNewStateCompl);  


	DEBUG_TRACE( cout << "obsPoss"<< endl; );
	DEBUG_TRACE( obsPoss.write(cout) << endl; );

	int currObservation = chooseFromDistribution(obsPoss, ((double)rand()/RAND_MAX));

	DEBUG_TRACE( cout << "currObservation "<< currObservation << endl; );

	//*************************************************************
	map<string, string> aa = problem->getActionsSymbols(currAction);
	//cout<<"CURRENT ACTION INDEX: "<<currAction<<" ";
	//cout<<aa["pOneAction"]<<endl;
	//cout<<"OBS ";
	map<string, string> bb = problem->getObservationsSymbols(currObservation);
	map<string, string> cc = problem->getFactoredObservedStatesSymbols(actStateCompl->sval);
	//cout<<cc["pTwo_0"]<<" "<<bb["obs_ballDp2S"]<<endl;
	//cout<<"BEFORE";
	for (int ii=0;ii<xDim;ii++){
	    //cout<<" BHOUT "<<bhout[ii];
	}
	for (int ii=0;ii<xDim;ii++){
	    //cout<<" FHOUT "<<fhout[ii];
	}
	//cout<<endl;
	engine->checkTerminal(cc["pTwo_0"],bb["obs_ballDp2S"],bhout,fhout);
	//cout<<"AFTER ";
	for (int ii=0;ii<xDim;ii++){
	    //cout<<" BHOUT "<<bhout[ii];
	}
	for (int ii=0;ii<xDim;ii++){
	    //cout<<" FHOUT "<<fhout[ii];
	}
	

	// now that we have the action, state of observed variable, and observation,
	// we can update the belief of unobserved variable
	if (timeIter == 0) {  // check to see if the initial X is a distribution or a known state
	    if (currBelSt->sval == -1) // special case for first time step where X is a distribution
		    nextBelSt = problem->beliefTransition->nextBelief(currBelSt->bvec, currBelX, currAction, currObservation, actNewStateCompl->sval);
	    else
		    nextBelSt = problem->beliefTransition->nextBelief(currBelSt, currAction, currObservation, actNewStateCompl->sval);
	} else 
	    nextBelSt = problem->beliefTransition->nextBelief(currBelSt, currAction, currObservation, actNewStateCompl->sval);
	//actual states
	currUnobsState = newUnobsState; //Y state, hidden
	actStateCompl->sval = actNewStateCompl->sval;
	copy(*actStateCompl->bvec, *actNewStateCompl->bvec);
	
	//belief states
	copy(*currBelSt->bvec, *nextBelSt->bvec);
	currBelSt->sval = nextBelSt->sval;

	// added to stop simulation when at terminal state
	if(problem->getIsTerminalState(*actStateCompl))
	{
	    // Terminal state
	    // reward all 0, transfer back to it self
	    // TODO Consider reward not all 0 case
	    //cout <<"Terminal State!! timeIter " << timeIter << endl;
	    break;
	}

    }


    return firstAction;
}


