#include "POMDPFileGenerator.h"
#include <stdexcept>
#include <fstream>
#include <boost/filesystem.hpp>

using boost::filesystem;

bool POMDPFileGenerator::generate(std::string& filePath){
  bool ret = false;
  if(m_indexMap.size()==0 || m_actionMap.size()==0 || m_rewardMap.size()==0 || m_transitionMap.size()==0){
    throw std::exception("Either of State/Action/Reward/Transition is empty!");
    return false;
  }
  try{
  std::ofstream fs(filePath.c_str(),std::ios_base::out);
  fs.open();
  
  // Writing fileheader
  std::time tm;
  std::time_t result = std::time(NULL);
  boost::filesystem::path myPath(filePath);
  fs << "################################################################" << std::endl;
  fs << "#### FILENAME     : " << myPath.filename() << std::endl;
  fs << "#### GENERATED ON : " << std::asctime(std::localtime(&result)) << std::endl;
  fs << "#### GENERATED BY : " << getName() << std::endl;
  
  // Writing file body
  
  fs.close();
  ret = true;
  }catch(std::exception ex){
  }
  return ret;
}
