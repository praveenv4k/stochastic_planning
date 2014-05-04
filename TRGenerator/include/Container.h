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

//Part of the functions in the Utils class are taken from 
//https://bitbucket.org/kritisen/utilitiescpp
class Utils{
public:
  static void ind2sub(std::vector<int> siz, int idx, std::vector<int>& sub){
    size_t N = siz.size();
    int *prod = new int [N];
    for (int i = 0; i < N; i++){
      prod[i] = 1;
      for (int j = N-1; j > i; j--){
	prod[i] *= siz[j];	
      }
    }
    sub.resize(N);
    for (int i = 0; i < N; i++){
      sub[i] = idx ;
      for (int j = 0; j < i ; j++)
	sub[i] = sub[i] % prod[j];
      sub[i] = (int)floor( (float)sub[i] / prod[i] );
    }
    delete [] prod;
  }

  static int sub2ind(std::vector<int> siz, std::vector<int> sub){
    size_t N = siz.size();
    int idx = 0;
    for (int i = 0; i < N; i++)
    {
	    int prod = 1;
	    for (int j = N-1; j > i; j--)
		    prod *= siz[j];
	    idx += sub[i] * prod;
    }
    return idx;    
  }
  
};

#endif //__CONTAINER_H__
