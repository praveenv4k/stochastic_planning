***Stochastic Planner evaluation project***

**About**
* The aim of the project is to study the Stochastic planning for the specific iCub grasping of a moving object. Markov decision process and Partially Observable MDP planning methods are 
focused.

**Requirements** 

* CMake is required. 
* YARP (www.yarp.it) and iCub (http://www.icub.org/) modules are necessary to be installed
* jsoncpp library amalgamation source is included in the project

**Build steps**

* Open Terminal and change directory to the stochastic_planner directory
* mkdir build
* cd build
* cmake ..
* make

**Running the application**

1. kick up yarpserver
2. start iCub_SIM
3. start simCartesianSolver
4. launch iKinCartesianSolver --context simCartesianControl --part right_arm
5. start CartesianControl
6. start StochasticPlanner
7. start ObjectController