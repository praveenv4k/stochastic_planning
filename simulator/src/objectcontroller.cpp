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
  
  radius=20;
  xc=0;
  yc=65;
  zc=40;
  
  // Y- UP
  // X - Left/Right of Robot
  // Z - In front of the robot
  //Vector boxSize;
  boxSize.resize(3);
  boxSize[0]=1.0;
  boxSize[1]=0.1;
  boxSize[2]=0.5;
  
  //Vector boxPos;
  boxPos.resize(3);
  boxPos[0]=0.0;
  boxPos[1]=0.4;
  boxPos[2]=0.45;

  ball_radius = 0.02;
  ballPos.resize(3);
  ballPos[0] = 0;
  ballPos[1] = boxPos[1]+boxSize[1]/2+ball_radius;
  ballPos[2] = 0.25;
  
  //ball top (0,0.5,0.25)
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
    Bottle del_all("world del all"); 
    outputStatePort.write(del_all,reply); 
    
    std::string str = "world mk sbox ";
    str = str + boxSize.toString(-1,1).c_str();
    str = str + " " + boxPos.toString(-1,1).c_str();
    str = str + " 0.8 0.8 0.8";
    std::cout << str << std::endl;
//     Bottle create_box("world mk sbox 1 0.1 0.5 0.0 0.4 0.6 0.8 0.8 0.8");
    Bottle create_box(ConstString(str.c_str()));
    outputStatePort.write(create_box,reply);

#if STATIC
    Bottle create_obj("world mk ssph 0.02 0.0 1 0.2 0 1 0");
    outputStatePort.write(create_obj,reply);
#else
    str = "world mk sph ";
    str = str + "0.02";
    str = str + " " + ballPos.toString(-1,1).c_str();
    str = str + " 0 1 0";
    std::cout << str << std::endl;
//     Bottle create_box("world mk sbox 1 0.1 0.5 0.0 0.4 0.6 0.8 0.8 0.8");
    Bottle create_ball(ConstString(str.c_str()));
//     Bottle create_obj("world mk sph 0.02 0.0 1 0.2 0 1 0");
    outputStatePort.write(create_ball,reply);
#endif
    startTime = Time::now(); 
    //world mk sbox (three params for size) (three params for pos) (three params for colour) 
  }else{
        std::cout << "ObjectCtrl Thread did not start\n";
  }
}

void ObjectController::run() //Action &act, State &nxtState, bool &reached, bool &invalidAct)
{
#if STATIC
  Bottle move_obj("world set ssph 1"); 
  yarp::sig::Vector vec = getNextPosition();
  move_obj.addDouble(vec[0]); 
  move_obj.addDouble(vec[1]); 
  move_obj.addDouble(vec[2]);
  outputStatePort.write(move_obj,reply); 
#endif
}

yarp::sig::Vector ObjectController::getNextPosition(){
  //double x = m_currPosition[0] + m_VelX * m_Period;
  //double y = 
  double t = Time::now() - startTime;   
  double dx = (round(xc + radius*cos(t))/100); 
  double dy = (round(yc + radius*sin(t))/100); 
  double dz = round(zc)/100;
  m_currPosition[0] = dx;
  m_currPosition[1] = dy;
  m_currPosition[2] = dz;
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