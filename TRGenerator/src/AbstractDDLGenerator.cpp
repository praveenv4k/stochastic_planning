#include "AbstractDDLGenerator.h"
#include <stdexcept>
#include <iostream>
#include <fstream>
#include "Utils.h"

bool AbstractDDLGenerator::generate(std::string& filePath){
  bool ret = false;
  if(m_stateMap.size()==0 || m_actionMap.size()==0 || m_rewardMap.size()==0 || m_transitionMap.size()==0){
    throw std::logic_error("Either of State/Action/Reward/Transition is empty!");
    return false;
  }
  try{
  std::ofstream fs(filePath.c_str(),std::ios_base::out);
  
  // Write Header
  writeHeader(fs);
  
  // Writing file body
  writeBody(fs);
  
  // Writing file Footer
  writeFooter(fs);
  
  fs.close();
  ret = true;
  }catch(std::exception ex){
  }
  return ret;
}