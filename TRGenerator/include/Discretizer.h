#ifndef __DISCRETIZER_H__
#define __DISCRETIZER_H__

#include "Container.h"

template <typename T>
class Discretizer
{
    public:
        Discretizer (Container<T> min,Container<T> max,Container<T> step) : m_min( min ),m_max(max),m_step(step)
	{
	  
	}
        int operator() () { return 0; }
    private:
        Container<T> m_min;
	Container<T> m_max;
	Container<T> m_step;
};

#endif //__DISCRETIZER_H__
