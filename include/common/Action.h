/*! 
 *  \author    PraveenKumar Vasudevan
 *  \date      2014
 */
#ifndef __ACTION_H__
#define __ACTION_H__

#include "Container.h"

/**
 * @brief Action Class
 **/
class Action: public Container<double>{
  /**
   * @brief Gripper Status Enumeration
   **/
  enum Grasp{
  GripperOpen =1,
  GripperClose = 2
  };
};

#endif