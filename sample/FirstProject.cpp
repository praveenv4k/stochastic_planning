// FirstProject.cpp : Defines the entry point for the console application.
//

#include <stdio.h>
#include <math.h>
#include "yarp/os/all.h"
using namespace yarp::os; 

#if 0
int main() {

	yarp::os::Network yarp; 
	yarp::os::Bottle reply; 
	yarp::os::SystemClock clock;
	Port port; 
	port.open("/move_ball"); 
	yarp.connect("/move_ball","/icubSim/world"); 
	Bottle del_all("world del all"); 
	port.write(del_all,reply); 
	Bottle create_obj("world mk ssph 0.02 0.0 1 0.2 0 1 0");
	//Bottle create_box("world mk box 0.2 0.2 0.2 0.3 0.2 1 1 0 0");
	port.write(create_obj,reply);
	//Bottle box("world mk sbox 0.4 0.4 0.3 0.0 0.5976 0.15 1 0 0");
	Bottle box("world mk sbox 0.5 0.4 0.3 0.0 0.65 0.4 1 0 0");
	port.write(box,reply);
	double start = Time::now(); 
	int a=1,b=2;
	double root2 = sqrt(2);
	while (true) { 
		double t = Time::now() - start; 
		//double dx = cos(a*t); 
		//double dy = sin(b*t);
		double den = (sin(t)*sin(t))+1.0;
		double dx = a*root2*cos(t)/den;
		double dy = a*root2*cos(t)*sin(t)/den;
		Bottle move_obj("world set ssph 1"); 
		//move_obj.addDouble(dx/8); 
		//move_obj.addDouble(dy/8+1); 
		//move_obj.addDouble(1); 
		move_obj.addDouble(dx/8); 
		move_obj.addDouble(dy/8+0.75); 
		move_obj.addDouble(0.65);
		port.write(move_obj,reply); 
		clock.delay(0.1);
	} 
	return 0; 
} 

#else
int main() {

	yarp::os::Network yarp; 
	yarp::os::Bottle reply; 
	yarp::os::SystemClock clock;
	Port port; 
	port.open("/move_ball"); 
	yarp.connect("/move_ball","/icubSim/world"); 
	Bottle del_all("world del all"); 
	port.write(del_all,reply); 
	Bottle create_obj("world mk ssph 0.02 0.0 1 0.2 0 1 0");
	port.write(create_obj,reply);
	double start = Time::now(); 
	double radius=20,xc=0,yc=65,zc=40;
	while (true) { 
		double t = Time::now() - start; 
		
		Bottle move_obj("world set ssph 1"); 
		
		double dx = (round(xc + radius*cos(t))/100); 
		double dy = (round(yc + radius*sin(t))/100); 
		double dz = round(zc)/100;
    
		move_obj.addDouble(dx); 
		move_obj.addDouble(dy); 
		move_obj.addDouble(dz);
		port.write(move_obj,reply); 
		clock.delay(0.1);
	} 
	return 0; 
//         yarp::os::Network yarp; 
// 	yarp::os::Bottle reply; 
// 	yarp::os::SystemClock clock;
// 	Port port; 
// 	port.open("/move_arm"); 
// 	if(!yarp.connect("/move_arm","/actionPrimitivesMod/in")){
// 	  std::cout << "Cannot connect to actionPrimitivesMod" << std::endl;
// 	  return -1;
// 	}
// 	
// 	Bottle target;
// 	target.addDouble(0.0);
// 	target.addDouble(0.75);
// 	target.addDouble(0.4);
// 	port.write(target,reply);
// 	return 0; 
} 

#endif
