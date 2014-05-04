#ifndef __TYPES_H__
#define __TYPES_H__

#include <vector>
#include <map>
#include <boost/tuple/tuple.hpp>
#include <boost/unordered_map.hpp>

typedef boost::tuples::tuple<int,int> StateActionTuple;

struct StateActionHash
    : std::unary_function<StateActionTuple, std::size_t>
{
    std::size_t operator()(StateActionTuple const& e) const
    {
        std::size_t seed = 0;
        boost::hash_combine( seed, e.get<0>() );
        boost::hash_combine( seed, e.get<1>() );

        return seed;
    }
};

struct StateActionEqualTo
    : std::binary_function<StateActionTuple, StateActionTuple, bool>
{
    bool operator()(StateActionTuple const& x, StateActionTuple const& y) const
    {

        return (x.get<0>()==y.get<0>() &&
                x.get<1>()==y.get<1>());
    }
};

typedef boost::unordered_map<int,double> RewardMap;
//typedef std::map<std::pair<int,int>,std::vector<int> > TransitionMap;
typedef boost::unordered_map<StateActionTuple, std::vector<int>,StateActionHash,StateActionEqualTo > TransitionMap;


#endif //__TYPES_H__