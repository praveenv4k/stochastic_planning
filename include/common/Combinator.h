/*! 
 *  \author    PraveenKumar Vasudevan
 *  \date      2014
 */
#ifndef __COMBINATOR_H__
#define __COMBINATOR_H__

#include "Container.h"

template <typename T>
/**
 * @brief Combinator - Vector concatenation class
 **/
class Combinator
{
    public:
      /**
       * @brief Constructor
       *
       * @param x Container element to be suffixed
       **/
      Combinator(Container<T> x) : _x( x ) {
	}
	/**
	 * @brief Functor to concatenate
	 *
	 * @param x Container to be appened
	 * @return Container< T > - Concatenated vector
	 **/
	Container<T> operator() (Container<T> x){
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
