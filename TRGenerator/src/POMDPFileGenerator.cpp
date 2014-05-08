#include "POMDPFileGenerator.h"

bool POMDPFileGenerator::Generate(std::string& fileName){
  if(m_indexMap.size()==0 || m_rewardMap.size()==0 || m_transitionMap.size()==0){
    return;
  }
}
