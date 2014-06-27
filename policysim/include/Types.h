#ifndef __TYPES_H__
#define __TYPES_H__

#include <vector>
#include <map>
#include <boost/tuple/tuple.hpp>
#include <boost/functional/hash.hpp>
#include <boost/unordered_map.hpp>
#include "Utils.h"

struct VectorIndexHash;
struct VectorIndexEqualTo;

typedef vector< string > split_vector_type;
typedef boost::shared_ptr<yarp::sig::Vector> VectorPtr;
typedef boost::unordered_map<int,VectorPtr > IndexVectorMap;
typedef boost::unordered_map<VectorPtr,int,VectorIndexHash,VectorIndexEqualTo> VectorIndexMap;
typedef boost::shared_ptr<IndexVectorMap> IndexVectorMapPtr;
typedef boost::shared_ptr<VectorIndexMap> VectorIndexMapPtr;


struct VectorIndexHash
    : std::unary_function<VectorPtr, std::size_t>
{
    std::size_t operator()(VectorPtr const& e) const
    {
      std::size_t seed = 0;
      for(size_t i=0;i<e->size();i++){
	if(i==5) continue;
	boost::hash_combine(seed,e->operator[](i));
      }
      return seed;
    }
};

struct VectorIndexEqualTo
    : std::binary_function<VectorPtr, VectorPtr, bool>
{
    bool operator()(VectorPtr const& x, VectorPtr const& y) const
    {
      bool ret=true;
      for(size_t i=0;i<x->size();i++){
	  //ret = (*x == *y);
	  if(i==5) continue;
	  ret&= fabs(x->operator[](i)-y->operator[](i))<=1e-3;
      }
      return ret;
    }
};

struct StateIndexHash
    : std::unary_function<std::vector<double>, std::size_t>
{
    std::size_t operator()(std::vector<double> const& e) const
    {
//       std::size_t seed = 0;
//       for(size_t i=0;i<e.size();i++){
// 	if(i==5) continue;
// 	boost::hash_combine(seed,e[i]);
//       }
//       return seed;
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

typedef boost::unordered_map<std::vector<double>,int,StateIndexHash,StateIndexEqualTo> StateIndexMap;

#endif //__TYPES_H__