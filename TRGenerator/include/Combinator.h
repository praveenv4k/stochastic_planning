#ifndef __COMBINATOR_H__
#define __COMBINATOR_H__

#include "Container.h"

template <typename T>
class Combinator
{
    public:
        Combinator (Container<T> x) : _x( x ) {}
        Container<T> operator() (Container<T> x) 
	{
	  Container<T> temp;
 	  for(typename Container<T>::ContainerConstIter it=_x.begin();it!=_x.end();it++){
 	    temp.push_back(*it);
 	  }
 	  for(typename Container<T>::ContainerConstIter it=x.begin();it!=x.end();it++){
 	    temp.push_back(*it);
 	  }
 	  return temp;
	}
    private:
        Container<T> _x;
};

#endif //__COMBINATOR_H__
