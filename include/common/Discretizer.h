#ifndef __DISCRETIZER_H__
#define __DISCRETIZER_H__

#include "Container.h"
#include <math.h>
#include "Utils.h"
#include "AbstractDiscretizer.h"

template <typename T>
class Discretizer: public AbstractDiscretizer<T>
{
    public:
        Discretizer (Container<T> min,Container<T> max,Container<T> step) : 
	      m_min( min ),m_max(max),m_step(step),m_totalElements(0)
	{
	  initialize();
	}
	
	virtual size_t size() const{
	  return m_totalElements;
	}
        
        virtual int operator() () 
	{ 
	  id++;
	  return id;
	}

	virtual Container<T> getValueAtIndex(int id){
	  Container<int> indices;
	  Container<T> values;
	  Utils::ind2sub(m_numElements,id,indices);
	  size_t dim = m_min.size();
	  values.resize(dim);
	  for(size_t i=0;i<dim;i++){
	    values[i] = m_min[i] + m_step[i] * indices[i];
	  }
	  //std::cout << "Index : " << Utils::sub2ind(m_numElements,indices) << std::endl;
	  return values;
	}
	
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
	
	virtual void reset(){
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
	      m_numElements[i] = round((m_max[i]-m_min[i])/m_step[i])+1;
	      m_totalElements*=m_numElements[i];
	    }
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
