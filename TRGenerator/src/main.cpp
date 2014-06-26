#include "Global.h"
#include "Trajectory.h"
#include <iostream>
#include <fstream>
#include "Action.h"
#include "Config.h"
#include "Test.h"
#include "DomainExtractor.h"

int main(void){
  // Test the implemented datastructures
  Test::testAll();
  // Generator
  DomainExtractor extractor(Config::instance()->root);
  extractor.generate();
  return 0;
}





