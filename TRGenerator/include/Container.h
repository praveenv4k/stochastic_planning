#ifndef __CONTAINER_H__
#define __CONTAINER_H__

#include <iostream>
#include <vector>

template <typename T>
class Container:public std::vector<T>
{
public:
  typedef typename Container::const_iterator ContainerConstIter;
  void print(){
    for(ContainerConstIter it=this->begin();it!=this->end();it++){
     std::cout<< *it << " ";
    }
    std::cout << std::endl;
  }
};

enum Grasp{
  GripperOpen =1,
  GripperClose = 2
};

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

class Action: public Container<double>{
};

#endif //__CONTAINER_H__
