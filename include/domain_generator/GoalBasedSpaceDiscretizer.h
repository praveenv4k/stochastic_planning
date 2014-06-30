#ifndef __GOALBASEDSPACEDISCRETIZER_H__
#define __GOALBASEDSPACEDISCRETIZER_H__

#include "AbstractSpaceDiscretizer.h"
#include "Trajectory.h"
#include "json/json.h"
#include <stdexcept>
#include <string.h>
#include <vector>

class GoalBasedSpaceDiscretizer:public AbstractSpaceDiscretizer<double>{
  GoalBasedSpaceDiscretizer(TrajectoryDiscretizerPtr trajDiscPtr,Json::Value objectConfig)
    :m_trajDiscPtr(trajDiscPtr),m_objectConfig(objectConfig)
  {
    m_trajSamples = 10;
    m_radius = 0.03;
    m_graspDelta = 0.005;
    m_increment = 0.02;
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
    if(m_trajDiscPtr!=NULL){
      if(!m_objectConfig.isNull()){
	Json::Value samples = m_objectConfig["trajectory"]["samples"];
	if(!samples.isNull()){
	  m_trajSamples = samples.asInt();
	}
	Json::Value rad = m_objectConfig["radius"].asInt();
	if(!rad.isNull()){
	  m_radius = rad.asDouble()/100;
	}
	std::cout << "GoalBasedDiscretizer: Numpoints -> " << m_trajSamples << " Radius -> " << m_radius << std::endl;
      }
      std::vector<Container<double> > poses;
      if(m_trajDiscPtr->getAllPoses(10,poses)){
	for(size_t i=0;i<poses.size();i++){
// 	  Container<double> pose=
// 	  for(int j=0;i<10;j++){
// 	    Container<double> val;
// 	    val.resize(4);
// 	    val[0] = val[0] + j*m_increment*
// 	  }
	}
      }
    }
  }
  
  double m_radius;
  double m_graspDelta;
  double m_increment;
  int m_trajSamples;
  TrajectoryDiscretizerPtr m_trajDiscPtr;
  Json::Value m_objectConfig;
  std::map<int,Container<double> > m_valueMap;
};

#endif