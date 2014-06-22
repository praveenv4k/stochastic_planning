/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2014  <copyright holder> <email>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "objectcontroller.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


ObjectController::ObjectController(const double period):RateThread(int(period*1000))
{
  // Initializing the private members
  m_VelX = 0.1; // 0.1 m/s
  m_Mean = 0; // Zero mean //TODO
  m_Sigma = 0.005; // Standard Deviation in Position //TODO
  m_Period = period; // Period of the Thread;
  
  // Resizing the position vectors
  m_initPosition.resize(3);
  m_currPosition.resize(3);
  
  // Set the initial position. 
  // TODO Read from config file
  m_initPosition[0] = 0.0;
  m_initPosition[1] = 1;
  m_initPosition[2] = 0.4;
  
  m_currPosition = m_initPosition;
}

ObjectController::~ObjectController()
{

}

bool ObjectController::threadInit()
{
    std::string outputPortName = "/" ;
    outputPortName += getName();
    outputPortName += "/states:o";
    if (!outputStatePort.open(outputPortName.c_str()))
    {
        std::cout <<": unable to open port to send states\n";
        return false;  // unable to open; let RFModule know so that it won't run
    }

    /* Connect to iCub Simulation world */

    if(! Network::connect(outputPortName.c_str(),"/icubSim/world"))
    {
        std::cout <<": unable to connect to iCub Simulation world.\n";
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
//     Bottle& outputState = outputStatePort.prepare();
//     outputState.clear();
//     outputState.addString("world del all");
//     outputStatePort.writeStrict();
// 	
//     outputState = outputStatePort.prepare();
//     outputState.clear();
//     outputState.addString("world mk ssph 0.02 0.0 1 0.4 1 0 0");
//     outputStatePort.writeStrict();
	
    return true;
}

void ObjectController::threadRelease()
{
    outputStatePort.close();
}

void ObjectController::afterStart(bool s)
{
    if (s){
        std::cout << "ObjectCtrl Thread started successfully\n";
// 	Bottle& outputState = outputStatePort.prepare();
	outputState.clear();
	outputState.add("world del all");
	outputStatePort.write(outputState,reply);
	std::cout << outputState.toString() <<std::endl;
// 	outputStatePort.write();
	
// 	outputState = outputStatePort.prepare();
	outputState.clear();
	outputState.add("world mk ssph 0.02 0.0 1 0.4 1 0 0");
	outputStatePort.write(outputState,reply);
	std::cout << outputState.toString() <<std::endl;
// 	outputStatePort.write();
    }
    else{
        std::cout << "ObjectCtrl Thread did not start\n";
    }
}

void ObjectController::run() //Action &act, State &nxtState, bool &reached, bool &invalidAct)
{
    //Bottle& outputState = outputStatePort.prepare();
    
    Bottle outputState1("world set ssph 1");
    //outputState.clear();
    //outputState.addString("world set ssph 1");
    yarp::sig::Vector pos = getNextPosition();
    outputState1.addDouble(pos[0]);
    outputState1.addDouble(pos[1]);
    outputState1.addDouble(pos[2]);
    outputStatePort.write(outputState1,reply);
    std::cout << outputState1.toString() <<std::endl;
    
//     outputStatePort.write();
//     std::cout << "Data written to port successfully\n";
}

yarp::sig::Vector ObjectController::getNextPosition(){
  //double x = m_currPosition[0] + m_VelX * m_Period;
  //double y = 
  return m_currPosition;
}

std::string ObjectController::getPositionStr(yarp::sig::Vector& vector) const{
  std::string str;
  str += vector[0];
  str += " ";
  str += vector[1];
  str += " ";
  str += vector[2];
  return str;
}