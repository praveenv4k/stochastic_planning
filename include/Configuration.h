#ifndef __CONFIGURATION_H__
#define __CONFIGURATION_H__

#include "Global.h"

class Config{
private:
  static Config* m_pConfig;
  Config(){
    m_pConfig = 0;
  }
  Config(Config& config){
  }
  Config& operator=(Config const&){
  }
public:
  Json::Value root;
  bool hasCollided;
  
  static Config* instance();
};

#endif //__CONFIGURATION_H__