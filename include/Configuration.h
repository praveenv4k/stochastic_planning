#ifndef __CONFIGURATION_H__
#define __CONFIGURATION_H__

#include <new>      // To set handler for new
#include <limits>   // To set precision for streams
#include <iomanip>  // Some more on precision
#include <fstream>
#include <ostream>
#include <string>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <fstream>
#include <stack>
#include <map>
#include <tr1/unordered_map>

// YARP
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>

#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>

// iCub
 #include <iCub/iKin/iKinInv.h>


// GSL
#include <gsl/gsl_math.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_fit.h>
#include <gsl/gsl_sort.h>
#include <gsl/gsl_statistics.h>
#include <gsl/gsl_rng.h>



//Boost
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>     // For Graph pointers
#include <boost/tuple/tuple.hpp>    // For tuples
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include "boost/tuple/tuple.hpp"
#include <boost/graph/graphviz.hpp>
#include <boost/config.hpp>
#include <boost/concept/assert.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/random.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/program_options.hpp>

// OpenCV files
#include <cv.h>
#include <highgui.h>
#include <opencv2/highgui/highgui.hpp>

// Eigen
//#include <eigen3/Eigen/Dense>

// Jason
#include <json/json.h>
#define JINDEX (Json::Value::ArrayIndex)

#define EPSILON 1e-10
#define ACTION_DIM 5
#define PRIMITIVE_ACTION_DIM 1
#define STATE_DIM 4
#define MINUS_INF_Q (-50.0)
#define INVALID_STATE 0
#define W0 0
#define WBAD 1
#define WOK 2


inline timespec diff(timespec start, timespec end)
{
    timespec temp;
    if ((end.tv_nsec-start.tv_nsec)<0) {
        temp.tv_sec = end.tv_sec-start.tv_sec-1;
        temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
    } else {
        temp.tv_sec = end.tv_sec-start.tv_sec;
        temp.tv_nsec = end.tv_nsec-start.tv_nsec;
    }
    return temp;
}

namespace state
{
    enum {x,y,z,o};
}
namespace action
{
    enum {dX,dY,dZ,aV,dO};
}

#define PATH2CONFIG "../resources/conf/Config.json"
class Config
{

public:
    static Config* instance();
    static Config* refresh();
    Json::Value root;
    double totTime;
    bool hasCollided;

    ~Config()
    {
        instanceFlag = false;
    }
    static int GetVersion();
    static void SetVersion(int version);

private:
    static bool instanceFlag;
    static bool isParsed;
    static int m_version;
    static Config *single;      // Trivia: This is thread-safe only for GNU compiler and also for c++11,
                                // we dont give damn for airhockey as of now since its serial!
    Config()
    {
        Json::Reader rd;
        std::ifstream tst(PATH2CONFIG, std::ifstream::binary);
        isParsed = rd.parse(tst,root,false);
        m_version = 0;
        hasCollided = false;
    }
};

bool file_is_empty(std::ifstream& pFile);
#endif // __CONFIGURATION_H__
