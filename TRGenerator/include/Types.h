#ifndef __TYPES_H__
#define __TYPES_H__

#include <vector>
#include <map>
#include <boost/tuple/tuple.hpp>
#include <boost/functional/hash.hpp>
#include <boost/unordered_map.hpp>
#include "Utils.h"

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

struct StateIndexHash
    : std::unary_function<std::vector<double>, std::size_t>
{
    std::size_t operator()(std::vector<double> const& e) const
    {
	return boost::hash_range(e.begin(),e.end());
    }
};

struct StateIndexEqualTo
    : std::binary_function<std::vector<double>, std::vector<double>, bool>
{
    bool operator()(std::vector<double> const& x, std::vector<double> const& y) const
    {
      bool ret=true;
      for(size_t i=0;i<x.size();i++){
	  ret&= Utils::isEqual(x[i],y[i]);
      }
      return ret;
    }
};

//typedef boost::unordered_map<StateActionTuple,double,StateActionHash,StateActionEqualTo> RewardMap;
typedef boost::unordered_map<int,double> RewardMap;
//typedef std::map<std::pair<int,int>,std::vector<int> > TransitionMap;
typedef boost::unordered_map<StateActionTuple, std::vector<int>,StateActionHash,StateActionEqualTo > TransitionMap;
typedef boost::unordered_map<std::vector<double>,int,StateIndexHash,StateIndexEqualTo> StateIndexMap;

#endif //__TYPES_H__