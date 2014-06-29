#ifndef __ABSTRACT_DISCRETIZER_H__
#define __ABSTRACT_DISCRETIZER_H__

#include "Container.h"
#include <boost/shared_ptr.hpp>

template <typename T>
class AbstractSpaceDiscretizer{
public:
  AbstractSpaceDiscretizer(){
  }
  
  virtual size_t size() const{
    return m_totalElements;
  }
  
  virtual int operator() (){
    id++;
    return id;
  }
  
  virtual Container<T> getValueAtIndex(int id)=0;
  
  virtual bool isInLimits(Container<T> value){
    bool ret = true;
    if(value.size() == m_min.size()){
      for(size_t i=0;i<value.size();i++){
	if(value[i]-m_min[i] < 0 || m_max[i]-value[i] < 0){
	  ret = false;
	  break;
	}
      }
    }
    return ret;
  }
  virtual void reset()=0;
protected:
  Container<T> m_min;
  Container<T> m_max;
  size_t m_totalElements;
  int id;
};

typedef boost::shared_ptr<AbstractSpaceDiscretizer<double> > AbstractSpaceDiscretizerPtr;

#endif