#ifndef __CONTAINER_H__
#define __CONTAINER_H__

#include <iostream>
#include <vector>
#include <math.h>

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
  template <typename U>
  friend std::ostream& operator<< (std::ostream& stream, const Container<U>& container);
};

template <typename U>
std::ostream& operator <<(std::ostream& stream, const Container<U>& container) {
  int i=0;
  for(typename Container<U>::ContainerConstIter it=container.begin();it!=container.end();it++){
    if(i>0) stream << " ";
    stream<< *it;
    i++;
  }
  return stream;
}

enum Grasp{
  GripperOpen =1,
  GripperClose = 2
};


#endif //__CONTAINER_H__
