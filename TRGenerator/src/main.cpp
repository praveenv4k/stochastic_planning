#include "Global.h"
#include "Trajectory.h"
#include <iostream>
#include <fstream>
#include "Action.h"
#include "Config.h"
#include "Test.h"
#include "Generator.h"

int main(void){
  // Test the implemented datastructures
  Test::testAll();
  
  // Generator
  Generator generator(Config::instance()->root);
  generator.generate();
  
  return 0;
}





