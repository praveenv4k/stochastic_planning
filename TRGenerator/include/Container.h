#ifndef __CONTAINER_H__
#define __CONTAINER_H__

#include <iostream>
#include <vector>
#include <math.h>
#include <stdexcept>

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
  
  Container<T> operator*(const double val){
    Container<T> ret;
    ret.resize(this->size());
    for(size_t i=0;i<this->size();i++){
      ret[i]=this->operator[](i)*val;
    }
    return ret;
  }
  
  Container<T> operator/(const double val){
    if(fabs(val-0)<=1e-6){
      throw std::invalid_argument("Divide by zero warning!");
    }
    Container<T> ret;
    ret.resize(this->size());
    for(size_t i=0;i<this->size();i++){
      ret[i]=this->operator[](i)/val;
    }
    return ret;
  }
  
  Container<T> operator-(const Container<T>& c){
    if(this->size() != c.size()){
      throw std::invalid_argument("The vector dimensions do not agree!");
    }
    Container<T> ret;
    ret.resize(this->size());
    for(size_t i=0;i<this->size();i++){
      ret[i]=this->operator[](i)-c[i];
    }
    return ret;
  }
  
  Container<T> operator+(const Container<T>& c){
    if(this->size() != c.size()){
      throw std::invalid_argument("The vector dimensions do not agree!");
    }
    Container<T> ret;
    ret.resize(this->size());
    for(size_t i=0;i<this->size();i++){
      ret[i]=this->operator[](i)+c[i];
    }
    return ret;
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

template <typename U>
std::ostream& operator <<(std::ostream& stream, const std::vector<U>& container) {
  int i=0;
  for(typename std::vector<U>::const_iterator it=container.begin();it!=container.end();it++){
    if(i>0) stream << " ";
    stream<< *it;
    i++;
  }
  return stream;
}

#endif //__CONTAINER_H__
