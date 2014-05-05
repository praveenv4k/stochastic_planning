#ifndef __DISCRETIZER_H__
#define __DISCRETIZER_H__

#include "Container.h"
#include <math.h>
#include "Utils.h"

template <typename T>
class Discretizer
{
    public:
        Discretizer (Container<T> min,Container<T> max,Container<T> step) : 
	      m_min( min ),m_max(max),m_step(step),m_totalElements(0)
	{
	  initialize();
	}
	size_t size() const{
	  return m_totalElements;
	}
        int operator() () 
	{ 
	  id++;
	  //Utils::ind2sub(m_numElements,id,m_indices);
	  //m_indices.print();
	  return id;
	}
	Container<T> getValueAtIndex(int id){
	  Container<int> indices;
	  Container<T> values;
	  Utils::ind2sub(m_numElements,id,indices);
	  size_t dim = m_min.size();
	  values.resize(dim);
	  //indices.print();
	  for(size_t i=0;i<dim;i++){
	    values[i] = m_min[i] + m_step[i] * indices[i];
	  }
	  //std::cout << "Index : " << Utils::sub2ind(m_numElements,indices) << std::endl;
	  return values;
	}
	
	void reset(){
	  initialize();
	}
    private:
	void initialize(){
	  id = -1;
	  size_t dim = m_min.size();
	  if(dim>0){
	    m_numElements.resize(dim);
	    m_indices.resize(dim);
	    m_totalElements=1;
	    for(size_t i=0;i<dim;i++){
	      m_indices[i] = 0;
	      m_numElements[i] = floor((m_max[i]-m_min[i])/m_step[i])+1;
	      m_totalElements*=m_numElements[i];
	    }
#if 0
	    std::cout << "Total number of states: " << m_totalElements << std::endl;
	    std::cout << "Minimum Bounds: "; m_min.print();
	    std::cout << "Maximum Bounds: "; m_max.print();
	    std::cout << "Values at Index 0 : ";
	    Container<T> value0 = getValueAtIndex(0);
	    value0.print();
	    std::cout << "Values at Index " << m_totalElements << " : " ;
	    Container<T> valueMax = getValueAtIndex(m_totalElements);
	    valueMax.print();
#endif
	  }
	}
	Container<T> m_min;
	Container<T> m_max;
	Container<T> m_step;
	Container<int> m_numElements;
	Container<int> m_indices;
	size_t m_totalElements;
	int id;
};

#endif //__DISCRETIZER_H__
