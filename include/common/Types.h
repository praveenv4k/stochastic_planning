/*! 
 *  \author    PraveenKumar Vasudevan
 *  \date      2014
 */
#ifndef __TYPES_H__
#define __TYPES_H__

#include <vector>
#include <map>
#include <boost/tuple/tuple.hpp>
#include <boost/functional/hash.hpp>
#include <boost/unordered_map.hpp>
#include "Utils.h"

/**
 * @brief Arm status enumeration
 **/
enum armstatus_t {
  IDLE,
  REACHED,
  MOVING
};

/**
 * @brief Grasp status enumeration
 **/
enum gstatus_t {
  RELEASING,
  RELEASED,
  GRASPED,
  GRASPING
};

/**
 * @brief State Action Tuple
 **/
typedef boost::tuples::tuple<int,int> StateActionTuple;

/**
 * @brief State Action Hashing function
 **/
struct StateActionHash
    : std::unary_function<StateActionTuple, std::size_t>
{
  /**
   * @brief Hashing operator
   *
   * @param e StateActionTuple whose hash has to be computed
   * @return :size_t -  computed hash
   **/
  std::size_t operator()(StateActionTuple const& e) const
    {
        std::size_t seed = 0;
        boost::hash_combine( seed, e.get<0>() );
        boost::hash_combine( seed, e.get<1>() );

        return seed;
    }
};

/**
 * @brief State Action Tuple Equality operator
 **/
struct StateActionEqualTo
    : std::binary_function<StateActionTuple, StateActionTuple, bool>
{
  /**
   * @brief StateActionTuple equality checking operator
   *
   * @param x StateActionTuple
   * @param y StateActionTuple
   * @return bool - true if equal, false otherwise
   **/
  bool operator()(StateActionTuple const& x, StateActionTuple const& y) const
    {

        return (x.get<0>()==y.get<0>() &&
                x.get<1>()==y.get<1>());
    }
};

/**
 * @brief StateIndex key hashing function
 **/
struct StateIndexHash
    : std::unary_function<std::vector<double>, std::size_t>
{
  /**
   * @brief Hashing operator
   *
   * @param e State Index key to be hashed
   * @return :size_t - Hash value
   **/
  std::size_t operator()(std::vector<double> const& e) const
    {
	return boost::hash_range(e.begin(),e.end());
    }
};

/**
 * @brief StateIndex equality check
 **/
struct StateIndexEqualTo
    : std::binary_function<std::vector<double>, std::vector<double>, bool>
{
  /**
   * @brief Equality checking operator
   *
   * @param x Vector
   * @param y Vector
   * @return bool - true if equal, otherwise false
   **/
    bool operator()(std::vector<double> const& x, std::vector<double> const& y) const
    {
      bool ret=true;
      for(size_t i=0;i<x.size();i++){
	  ret&= Utils::isEqual(x[i],y[i]);
      }
      return ret;
    }
};

/**
 * @brief Reward Matrix map (s,r)
 **/
typedef boost::unordered_map<int,double> RewardMap;
/**
 * @brief Collision map
 **/
typedef boost::unordered_map<int,bool> CollisionMap;
/**
 * @brief Transition Matrix map
 **/
typedef boost::unordered_map<StateActionTuple, std::vector<int>,StateActionHash,StateActionEqualTo > TransitionMap;
/**
 * @brief State Index Map
 **/
typedef boost::unordered_map<std::vector<double>,int,StateIndexHash,StateIndexEqualTo> StateIndexMap;

#endif //__TYPES_H__