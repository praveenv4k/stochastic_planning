#ifndef __CONTAINER_H__
#define __CONTAINER_H__

#include <iostream>
#include <vector>

template <typename T>
class Container:public std::vector<T>
{
public:
  void print(){
    for(typename Container::const_iterator it=this->begin();it!=this->end();it++){
     std::cout<< *it << " ";
    }
    std::cout << std::endl;
  }
};

#endif //__CONTAINER_H__
