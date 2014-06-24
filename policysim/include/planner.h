
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

using namespace std;
using boost::lexical_cast;
using boost::bad_lexical_cast;


typedef vector< string > split_vector_type;
    
typedef boost::shared_ptr<std::vector<double> > VectorPtr;
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
    }

    bool open(yarp::os::ResourceFinder &rf);

    bool close();
    void loop(); 
    bool interrupt();
    
    bool read_states(std::string states_file);
    bool read_actions(std::string actions_file);
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
private:
    yarp::os::BufferedPort<yarp::os::Bottle> plannerCmdPort;
    yarp::os::BufferedPort<yarp::os::Bottle> plannerStatusPort;
    std::queue<yarp::sig::Vector> posQueue;
    
    IndexVectorMapPtr m_States;
    IndexVectorMapPtr m_Actions;
    
    double t;
    double t0;
    double t1;
    bool first;
    bool sent;
};

   
   



   
