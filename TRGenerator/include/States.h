#ifndef __STATES_H__
#define __STATES_H__

#include "Container.h"

class State: public Container<double>{
public:
  State(size_t capacity){
    this->resize(capacity);
  }
  int getId() const{
    int id = 1;
    for(ContainerConstIter it=this->begin();it!=this->end();it++){
     id=id* *it;
    }
    return id;
  }
};

#endif