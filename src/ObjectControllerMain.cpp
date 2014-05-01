
#include "objectcontroller.h"

int main(void){
  Network yarp;

    if (!yarp.checkNetwork())
    {
        std::cout << "No yarp network, quitting\n";
        return -1;
    }

    ObjectController myThread(4000); //period is 40ms

    myThread.start();

//     bool done=false;
//     double startTime=Time::now();
//     while(!done)
//     {
//         if ((Time::now()-startTime)>5)
//             done=true;
//     }
//     
//     myThread.stop();

    return 0;
}
