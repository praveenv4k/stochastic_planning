#ifndef __COMBINATOR_H__
#define __COMBINATOR_H__

#include "Container.h"

template <typename T>
class Combinator
{
    public:
        Combinator (Container<T> x) : _x( x ) {}
        int operator() (Container<T> x) { return 0; }
    private:
        Container<T> _x;
};

#endif //__COMBINATOR_H__
