#ifndef __DISCRETIZER_H__
#define __DISCRETIZER_H__

#include "Container.h"

template <typename T>
class Discretizer
{
    public:
        Discretizer (Container<T> x) : _x( x ) {}
        int operator() () { return 0; }
    private:
        Container<T> _x;
};

#endif //__DISCRETIZER_H__
