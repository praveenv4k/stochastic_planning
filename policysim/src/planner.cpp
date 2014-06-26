
#include "planner.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

#include <math.h>
#include <fstream>

#define DEBUG_TRACE_ON 1

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
      cout << " Best Action : " << action <<  " Next State: " << currBelSt->sval;
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
//     for(int i=0;i<10;i++){
//       int prevState = currBelSt->sval;
//       cout << " Current State : " << currBelSt->sval;
//       action = runFor(1,NULL,reward,expReward);
//       cout << " Best Action : " << action <<  " Next State: " << currBelSt->sval;
//       if(action!=-1 && prevState!=currBelSt->sval){
// 	VectorPtr v=m_States->operator[](currBelSt->sval);
// 	Vector v1;
// 	v1.resize(4);
// 	v1[0]= v->operator[](0)/100;
// 	v1[1]= v->operator[](1)/100;
// 	v1[2]= v->operator[](2)/100;
// 	v1[3]= 1;
// 	posQueue.push(v1);
//       }else{
// 	break;
//       }
//     }
    cout << " Reward: " << reward << " Expected Reward: " << expReward << std::endl;

    printf("Targets count: %d\n",posQueue.size());
    t=t1=t0 = Time::now();
    return ret;
}

// bool Planner::get_state(int index,Vector& vec){
//   return true;
// }
// 
// bool Planner::get_action(int index,Vector& vec){
//   return true;
// }

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

bool Planner::read_states(std::string states_file){
  bool bret = false;
  if(!states_file.empty()){
    std::ifstream fs(states_file.c_str(),std::ifstream::in);
    string str;
    while(std::getline(fs,str)){
      std::vector<double> vec;
      if(!str.empty()){
	if(parse_state_action(str,vec)){
// 	  VectorPtr ptr(new std::vector<double>());
	  VectorPtr ptr(new yarp::sig::Vector());
	  for(size_t i=1;i<vec.size()-1;i++){
	    ptr->push_back(vec[i]);
	  }
	  m_States->insert(std::pair<int,VectorPtr>((int)vec[0],ptr));
	}
      }
    }
    bret = true;
  }
  std::cout << "Number of states: " << m_States->size() <<std::endl;
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
// 	  VectorPtr ptr(new std::vector<double>());
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
    actStateCompl->sval = chooseFromDistribution(*startBeliefX);
    copy(currBelX, *startBeliefX);
  }
  else
  {
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
  
//   double reward,expReward;
//   for(int i=0;i<5;i++){
//     cout << " Current State : " << currBelSt->sval;
//     int action = runFor(1,NULL,reward,expReward);
//     cout << " Best Action : " << action <<  " Next State: " << currBelSt->sval;
//   }
//   cout << " Reward: " << reward << " Expected Reward: " << expReward << std::endl;
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


