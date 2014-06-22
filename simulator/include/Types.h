#ifndef __TYPES_H__
#define __TYPES_H__

#include <vector>
#include <map>

typedef std::map<int,double> RewardMap;
//typedef std::map<std::pair<int,int>,std::vector<int> > TransitionMap;
typedef std::map<int, std::vector<int> > TransitionMap;

enum armstatus_t {
  IDLE,
  REACHED,
  MOVING
};

enum gstatus_t {
  RELEASED,
  GRASPED,
  BUSY
};

#endif //__TYPES_H__