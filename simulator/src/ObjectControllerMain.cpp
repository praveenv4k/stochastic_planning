
#include "ObjectControllerModule.h"

int main(void){
  Network yarp;
  if (!yarp.checkNetwork()){
    std::cout << "Error: yarp server does not seem available\n";
    return -1;
  }
  ObjectControllerModule mod; 
  ResourceFinder rf;
  return mod.runModule(rf);
}
