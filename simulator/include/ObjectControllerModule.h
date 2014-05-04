#include "Global.h"
#include "objectcontroller.h"

#define CTRL_THREAD_PER 0.02 // 20 ms

class ObjectControllerModule:public yarp::os::RFModule
{
protected:
  ObjectController* m_pCtrlThread;
public:
  virtual bool configure(ResourceFinder &rf){
    yarp::os::Time::turboBoost();
    
    m_pCtrlThread = new ObjectController(CTRL_THREAD_PER);
    
    if(!m_pCtrlThread->start()){
      delete m_pCtrlThread;
      return false;
    }
    return true;
  }
  
  virtual bool close(){
    if(m_pCtrlThread!=0){
      m_pCtrlThread->stop();
      delete m_pCtrlThread;
      m_pCtrlThread = 0;
    }
    return true;
  }
  
  virtual double getPeriod(){
    return 1.0;
  }
  
  virtual bool updateModule(){
    return true;
  }
};