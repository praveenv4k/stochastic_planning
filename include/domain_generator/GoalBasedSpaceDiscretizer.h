/*! 
 *  \author    PraveenKumar Vasudevan
 *  \date      2014
 */
#ifndef __GOALBASEDSPACEDISCRETIZER_H__
#define __GOALBASEDSPACEDISCRETIZER_H__

#include "AbstractSpaceDiscretizer.h"
#include "Trajectory.h"
#include "json/json.h"
#include <stdexcept>
#include <string.h>
#include <vector>

/**
 * @brief Goal Based Greedy discretizer
 **/
class GoalBasedSpaceDiscretizer:public AbstractSpaceDiscretizer<double>{
public:
  /**
   * @brief Constructor
   *
   * @param trajDiscPtr Object trajectory pointer
   * @param objectConfig Object configuration information
   **/
  GoalBasedSpaceDiscretizer(TrajectoryDiscretizerPtr trajDiscPtr,Json::Value objectConfig)
    :m_trajDiscPtr(trajDiscPtr),m_objectConfig(objectConfig)
  {
    m_trajSamples = 10;
    m_radius = 3;
    m_graspDelta = 0.5;
    m_increment = 3;
    initialize();
  }
  
  /**
   * @brief Get the spatial vector at a given index
   *
   * @param id Spatial index
   * @return Container< double > - Element at index id
   **/
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
  
  /**
   * @brief Resets the index
   *
   * @return void
   **/
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
	  //m_radius = rad.asDouble()/100;
	  m_radius = rad.asDouble();
	}
	std::cout << "GoalBasedDiscretizer: Numpoints -> " << m_trajSamples << " Radius -> " << m_radius << std::endl;
      }
      std::vector<Container<double> > poses;
      int index=0;
      if(m_trajDiscPtr->getAllPoses(10,poses)){
	std::cout << "Numpoints :" << poses.size() << std::endl;
	for(size_t i=0;i<poses.size();i++){
	  Container<double> pose=poses[i];
	  for(int j=0;j<10;j++){
	    Container<double> val;
	    val.resize(4);
	    val[0] = pose[0];
	    val[1] = pose[1]+m_radius + j*m_increment;
	    val[2] = pose[2];
	    if(j==0){
	      val[1]+=m_graspDelta;
	    }
	    val[3]=0;
#if 0
	    std::cout << val << std::endl;
#endif
	    m_valueMap.insert(std::pair<int,Container<double> >(index,val));
	    val[3]=1;
#if 0
	    std::cout << val << std::endl;
#endif
	    index++;
	    m_valueMap.insert(std::pair<int,Container<double> >(index,val));
	    index++;
	  }
	}
      }
    }
    std::cout << "Goal Disc: " << m_valueMap.size() << std::endl;
    this->m_totalElements=m_valueMap.size();
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