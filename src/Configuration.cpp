#include "Configuration.h"

Config* Config::m_pConfig=NULL;

Config* Config::instance(){
    if(m_pConfig==0){
      m_pConfig = new Config();
    }
    return m_pConfig;
  }