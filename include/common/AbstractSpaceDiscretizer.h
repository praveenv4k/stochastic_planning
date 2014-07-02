/*! 
 *  \author    PraveenKumar Vasudevan
 *  \date      2014
 */
#ifndef __ABSTRACT_DISCRETIZER_H__
#define __ABSTRACT_DISCRETIZER_H__

#include "Container.h"
#include <boost/shared_ptr.hpp>

template <typename T>
/**
 * @brief AbstractSpaceDiscretizer class
 **/
class AbstractSpaceDiscretizer{
public:
  /**
   * @brief Constructor
   *
   **/
  AbstractSpaceDiscretizer(){
  }
  
  /**
   * @brief Total number of disretized elements in space
   *
   * @return size_t - Size
   **/
  virtual size_t size() const{
    return m_totalElements;
  }
  
  /**
   * @brief Functor to increment the spatial index
   *
   * @return int - Index
   **/
  virtual int operator() (){
    id++;
    return id;
  }
  
  /**
   * @brief Get the value of the spatial element at index id
   *
   * @param id Index
   * @return Container< T > - Element at index id
   **/
  virtual Container<T> getValueAtIndex(int id)=0;
  
  /**
   * @brief Check if a given element is within the bounds of the space
   *
   * @param value Element to be checked for
   * @return bool - Exist/Not exist
   **/
  virtual bool isInLimits(Container<T> value){
    bool ret = true;
    if(value.size() == m_min.size()){
      for(size_t i=0;i<value.size();i++){
	if(value[i]-m_min[i] < 0 || m_max[i]-value[i] < 0){
	  ret = false;
	  break;
	}
      }
    }
    return ret;
  }
  /**
   * @brief Reset the index and reinitialize the spatial discretization
   *
   * @return void
   **/
  virtual void reset()=0;
protected:
  Container<T> m_min;
  Container<T> m_max;
  size_t m_totalElements;
  int id;
};

/**
 * @brief Pointer to the AbstractSpaceDiscretizer
 **/
typedef boost::shared_ptr<AbstractSpaceDiscretizer<double> > AbstractSpaceDiscretizerPtr;

#endif