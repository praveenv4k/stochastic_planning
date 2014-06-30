#ifndef __GOALBASEDSPACEDISCRETIZER_H__
#define __GOALBASEDSPACEDISCRETIZER_H__

#include "AbstractSpaceDiscretizer.h"
#include "Trajectory.h"
#include "json/json.h"
#include <stdexcept>
#include <string.h>

class GoalBasedSpaceDiscretizer:public AbstractSpaceDiscretizer<double>{
  GoalBasedSpaceDiscretizer(TrajectoryDiscretizerPtr trajDiscPtr,Json::Value objectConfig)
    :m_trajDiscPtr(trajDiscPtr),m_objectConfig(objectConfig)
  {
    initialize();
  }
  
  virtual Container<double> getValueAtIndex(int id){
    std::map<int,Container<double> >::iterator it = m_valueMap.find(id);
    if(it!=m_valueMap.end()){
      return m_valueMap[id];
    }
    else{
      throw new std::logic_error(std::string("Value at requested index does not exist"));
    }
    return Container<double>();
  }
  
  virtual void reset(){
    id=-1;
  }
  
private:
  void initialize(){
    id=-1;
  }
  
  TrajectoryDiscretizerPtr m_trajDiscPtr;
  Json::Value m_objectConfig;
  std::map<int,Container<double> > m_valueMap;
};

#endif