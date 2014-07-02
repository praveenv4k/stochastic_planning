/*! 
 *  \author    PraveenKumar Vasudevan
 *  \date      2014
 */
#ifndef __UNIFORM_SPACE_DISCRETIZER_H__
#define __UNIFORM_SPACE_DISCRETIZER_H__

#include "Container.h"
#include <math.h>
#include "Utils.h"
#include "AbstractSpaceDiscretizer.h"

template <typename T>
/**
 * @brief Uniform space discretization class - Extends abstract discretizer
 **/
class UniformSpaceDiscretizer: public AbstractSpaceDiscretizer<T>
{
public:
  /**
   * @brief Constructor
   *
   * @param min min bounds
   * @param max max bounds
   * @param step discretization steps
   **/
  UniformSpaceDiscretizer(Container<T> min,Container<T> max,Container<T> step)
  : m_step(step)
  {
    this->m_min = min;
    this->m_max = max;
    this->m_totalElements = 0;
    initialize();
  }

  /**
   * @brief Get the value of the element at index
   *
   * @param id Spatial index
   * @return Container< T > - Vector at index id
   **/
  virtual Container<T> getValueAtIndex(int id){
    Container<int> indices;
    Container<T> values;
    Utils::ind2sub(m_numElements,id,indices);
    size_t dim = this->m_min.size();
    values.resize(dim);
    for(size_t i=0;i<dim;i++){
      values[i] = this->m_min[i] + m_step[i] * indices[i];
    }
    return values;
  }

  /**
   * @brief Bound check
   *
   * @param value Value to be checked
   * @return bool - true if exists inside the space,false otherwise
   **/
  virtual bool isInLimits(Container<T> value){
    bool ret = true;
    if(value.size() == this->m_min.size()){
      for(size_t i=0;i<value.size();i++){
	if(value[i]-this->m_min[i] < 0 || this->m_max[i]-value[i] < 0){
	  ret = false;
	  break;
	}
      }
    }
    return ret;
  }

  /**
   * @brief Reset the spatial index
   *
   * @return void
   **/
  virtual void reset(){
    initialize();
  }

private:
  void initialize(){
    this->id = -1;
    size_t dim = this->m_min.size();
    if(dim>0){
      m_numElements.resize(dim);
      m_indices.resize(dim);
      this->m_totalElements=1;
      for(size_t i=0;i<dim;i++){
	m_indices[i] = 0;
	m_numElements[i] = round((this->m_max[i]-this->m_min[i])/m_step[i])+1;
	this->m_totalElements*=m_numElements[i];
      }
    }
  }
  Container<T> m_step;
  Container<int> m_numElements;
  Container<int> m_indices;
};

#endif //__DISCRETIZER_H__
