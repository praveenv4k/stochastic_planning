#include "Global.h"
#include "Trajectory.h"
#include <iostream>
#include <fstream>
#include "Action.h"
#include "Config.h"
#include "Test.h"
#include "DomainExtractor.h"
#include "ElapsedTime.h"

int main(void){
  // Test the implemented datastructures
  Test::testAll();
  return 0;
  // Generator
  ElapsedTime elapse(std::string("Domain Model Generation"));
  DomainExtractor extractor(Config::instance()->root);
  extractor.generate();
  return 0;
}





