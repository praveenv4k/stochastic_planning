/*! 
 *  \author    PraveenKumar Vasudevan
 *  \date      2014
 */
#ifndef __OBJECT_CONTROLLER_MODULE_H__
#define __OBJECT_CONTROLLER_MODULE_H__

#include "Global.h"
#include "objectcontroller.h"

//#define CTRL_THREAD_PER 0.02 // 20 ms
#define CTRL_THREAD_PER 5 // 20 ms

/**
 * @brief Object controller module extends RF module
 **/
class ObjectControllerModule:public yarp::os::RFModule
{
protected:
  ObjectController* m_pCtrlThread;
public:
  /**
   * @brief Constructor
   *
   **/
  ObjectControllerModule(){
    m_pCtrlThread=new ObjectController(CTRL_THREAD_PER);
  }
  
  /**
   * @brief Destructor
   *
   **/
  virtual ~ObjectControllerModule(){
    if(m_pCtrlThread!=NULL){
      delete m_pCtrlThread;
      m_pCtrlThread = NULL;
    }
  }
  /**
   * @brief Configure the module
   *
   * @param rf Resource finder interface
   * @return bool - Success/Failure
   **/
  virtual bool configure(ResourceFinder &rf){
    yarp::os::Time::turboBoost();
    return m_pCtrlThread->open(rf);
  }
  
  /**
   * @brief Close the module
   *
   * @return bool - Success/Failure
   **/
  virtual bool close(){
    return m_pCtrlThread->close();
  }
  
  /**
   * @brief Period at which the module has to be run
   *
   * @return double - Period in ms
   **/
  virtual double getPeriod(){
    return 0.1;
  }
  
  /**
   * @brief Control loop of the module - called every period returned by getPeriod() function
   *
   * @return bool
   **/
  virtual bool updateModule(){
    m_pCtrlThread->loop();
    return true;
  }
};

#endif