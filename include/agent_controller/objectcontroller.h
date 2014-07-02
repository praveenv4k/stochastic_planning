/*! 
 *  \author    PraveenKumar Vasudevan
 *  \date      2014
 */
#ifndef OBJECTCONTROLLER_H
#define OBJECTCONTROLLER_H

#include "Global.h"
#include "Container.h"

using namespace yarp::os;

/**
 * @brief The Object controller class which controls the motion of the object in the environment
 **/
class ObjectController
{
public:
  /**
   * @brief Constructor
   *
   * @param period Period of the thread
   **/
  ObjectController(const double period);
  /**
   * @brief Destructor
   *
   **/
  virtual ~ObjectController();
  /**
   * @brief Name of the controller
   *
   * @return :string - Controller Name
   **/
  std::string getName() const{
      return "objectCtrl";
    }
    /**
     * @brief Get the next position of the object to be set onto the world
     *
     * @return Container< double > - Position vector (3D)
     **/
    Container<double> getNextPosition();
    /**
     * @brief Open the device and ports
     *
     * @param rf Resource/Configuration information finder object
     * @return bool - Success/Failure
     **/
    bool open(yarp::os::ResourceFinder &rf);
    /**
     * @brief Close the ports/deinitialize
     *
     * @return bool - Success/Failure
     **/
    bool close();
    /**
     * @brief Control Loop
     *
     * @return void
     **/
    void loop(); 
    /**
     * @brief Interrupt the thread
     *
     * @return bool - Success.Failure
     **/
    bool interrupt();
private:
    ObjectController(const ObjectController& other)//:RateThread(10)
    {
    }
    virtual ObjectController& operator=(const ObjectController& other){
      return *this;
    }
    virtual bool operator==(const ObjectController& other) const{
    }

    Port outputStatePort;
    Container<double> m_currPosition;
    Container<double> m_exactPosition;
    Container<double> m_currElbowPosition;
    Container<double> m_initPosition;
    
    yarp::os::Bottle outputState;
    yarp::os::Bottle reply;
    
    yarp::os::Network yarp; 
    
    double m_VelX;
    double m_Mean;
    double m_Sigma;
    int m_Period;
    
    double startTime;
    double xc,yc,zc,radius;
        
    int m_currStep;
 
    double ball_radius;
    int m_multiple;
    int m_currmult;
    
    bool m_stop;
    bool m_static;
    
    double m_radius;
    double m_elbowRadius;
    int m_numPoints;
    bool m_elbowEnabled;
    bool m_noiseEnabled;
    
    std::vector<Container<double> > m_objPoses;
    std::vector<Container<double> > m_noisyObjPoses;
    std::vector<Container<double> > m_elbowPoses;
    
    yarp::os::BufferedPort<yarp::os::Bottle> objectCmdPort;
    yarp::os::BufferedPort<yarp::os::Bottle> objectStatusPort;
};

#endif // OBJECTCONTROLLER_H
