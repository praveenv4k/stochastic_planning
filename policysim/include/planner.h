
#include <string>
#include <vector>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Rand.h>
#include <yarp/os/Time.h>

#include <queue>
#include <map>
#include <vector>

#include <boost/unordered_map.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/find_iterator.hpp>
#include <boost/lexical_cast.hpp>


#include "SimulationEngine.h"
#include "GlobalResource.h"
#include "SimulationRewardCollector.h"
#include <string>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <ctime>

#include "CPTimer.h"

#ifdef _MSC_VER
#else
//for timing
#include <sys/param.h>
#include <sys/types.h>
#include <sys/times.h>
//end for timing
#endif

#include "MOMDP.h"
#include "ParserSelector.h"
#include "AlphaVectorPolicy.h"

using namespace momdp;

using namespace std;
using boost::lexical_cast;
using boost::bad_lexical_cast;


typedef vector< string > split_vector_type;
    
// typedef boost::shared_ptr<std::vector<double> > VectorPtr;
typedef boost::shared_ptr<yarp::sig::Vector> VectorPtr;
typedef boost::unordered_map<int,VectorPtr > IndexVectorMap;
typedef boost::shared_ptr<IndexVectorMap> IndexVectorMapPtr;

class Planner
{
   
public:

    Planner()
    {
      first = true;
      sent=false;
      m_States = IndexVectorMapPtr(new IndexVectorMap());
      m_Actions = IndexVectorMapPtr(new IndexVectorMap());
      actNewStateCompl = SharedPointer<BeliefWithState>(new BeliefWithState());
      actStateCompl = SharedPointer<BeliefWithState>(new BeliefWithState());
      currBelSt = SharedPointer<BeliefWithState>(new BeliefWithState());
      nextBelSt = SharedPointer<BeliefWithState>(new BeliefWithState());
      engine = boost::shared_ptr<SimulationEngine>(new SimulationEngine());
      rewardCollector = boost::shared_ptr<SimulationRewardCollector>(new SimulationRewardCollector());
      
      xDim = 3;
      bhout.resize(xDim);
      fhout.resize(xDim);
      
      mult = 1;
    }

    bool open(yarp::os::ResourceFinder &rf);

    bool close();
    void loop(); 
    bool interrupt();
    
    bool read_states(std::string states_file);
    bool read_actions(std::string actions_file);
    bool read_policy(std::string domain_file, std::string policy_file);
    bool parse_state_action(std::string str,std::vector<double>& vec){
      try{
	if(!str.empty()){
	split_vector_type SplitVec; // #2: Search for tokens
	boost::split( SplitVec, str, boost::is_any_of(" "),boost::token_compress_on ); // SplitVec == { "hello abc","ABC","aBc goodbye" }
	  for(size_t i=0;i<SplitVec.size();i++){
	    vec.push_back(lexical_cast<double>(SplitVec[i]));
	  }
	}
      }
      catch(std::exception& ex){
	std::cout << ex.what() << std::endl;
      }
      return true;
    }
    
    
    template <typename T>
    static double computeL2norm(T v1,T v2){
      size_t dim = v1.size();  
      if(dim != v2.size()){
	throw std::invalid_argument("The vector dimensions do not agree!");
      }
      double sum = 0;
      for(size_t i=0;i<dim;i++){
	double temp = v1[i]-v2[i];
	sum = sum+ temp*temp;
      }
      return sqrt(sum);
    }

    bool initialize_plan();
    int runFor(int iters, ofstream* streamOut, double& reward, double& expReward);
    
//     bool get_state(yarp::sig::Vector& vec);
//     bool get_action(yarp::sig::Vector& vec);
private:
    yarp::os::BufferedPort<yarp::os::Bottle> plannerCmdPort;
    yarp::os::BufferedPort<yarp::os::Bottle> plannerStatusPort;
    yarp::os::BufferedPort<yarp::os::Bottle> objectCmdPort;
    std::queue<yarp::sig::Vector> posQueue;
    
    IndexVectorMapPtr m_States;
    IndexVectorMapPtr m_Actions;
    
    SolverParams* solverParams;
    SharedPointer<MOMDP> problem;
    SharedPointer<AlphaVectorPolicy> policy;
    
    
    SharedPointer<BeliefWithState> actStateCompl;
    SharedPointer<BeliefWithState> actNewStateCompl;

    // policy follower state
    // belief with state
    SharedPointer<BeliefWithState> nextBelSt;
    SharedPointer<BeliefWithState> currBelSt;// for policy follower based on known x value
								      // set sval to -1 if x value is not known
    // belief over x. May not be used depending on model type and commandline flags, but declared here anyways.
    DenseVector currBelX; // belief over x
    
    
    boost::shared_ptr<SimulationEngine> engine;
    boost::shared_ptr<SimulationRewardCollector> rewardCollector;
    
    int xDim;
    vector<int> bhout;
    vector<int> fhout;
    
    int currAction;
    double currReward;
    
    int currUnobsState;
    int belSize;
    double mult;
    
    double t;
    double t0;
    double t1;
    bool first;
    bool sent;   
};

   
   



   
