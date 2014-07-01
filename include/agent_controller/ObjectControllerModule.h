#include "Global.h"
#include "objectcontroller.h"

//#define CTRL_THREAD_PER 0.02 // 20 ms
#define CTRL_THREAD_PER 5 // 20 ms

class ObjectControllerModule:public yarp::os::RFModule
{
protected:
  ObjectController* m_pCtrlThread;
public:
  ObjectControllerModule(){
    m_pCtrlThread=new ObjectController(CTRL_THREAD_PER);
  }
  
  virtual ~ObjectControllerModule(){
    if(m_pCtrlThread!=NULL){
      delete m_pCtrlThread;
      m_pCtrlThread = NULL;
    }
  }
  virtual bool configure(ResourceFinder &rf){
    yarp::os::Time::turboBoost();
    return m_pCtrlThread->open(rf);
  }
  
  virtual bool close(){
    return m_pCtrlThread->close();
  }
  
  virtual double getPeriod(){
    return 0.1;
  }
  
  virtual bool updateModule(){
    m_pCtrlThread->loop();
    return true;
  }
};