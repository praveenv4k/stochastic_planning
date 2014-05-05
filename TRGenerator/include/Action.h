#ifndef __ACTION_H__
#define __ACTION_H__

#include "Container.h"

class Action: public Container<double>{
  enum Grasp{
  GripperOpen =1,
  GripperClose = 2
  };
};

#endif