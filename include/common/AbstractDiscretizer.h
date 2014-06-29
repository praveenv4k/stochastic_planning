#ifndef __ABSTRACT_DISCRETIZER_H__
#define __ABSTRACT_DISCRETIZER_H__

#include "Container.h"

template <typename T>
class AbstractDiscretizer{
protected:
  AbstractDiscretizer(){
  }
public:
  virtual size_t size() const = 0;
  virtual int operator() () =0;
  virtual Container<T> getValueAtIndex(int id)=0;
  virtual bool isInLimits(Container<T> value)=0;
  virtual void reset()=0;
};

#endif