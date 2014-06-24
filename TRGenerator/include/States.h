#ifndef __STATES_H__
#define __STATES_H__

#include "Container.h"

class State: public Container<double>{
public:
  State(size_t capacity){
    this->resize(capacity);
  }
};

#endif