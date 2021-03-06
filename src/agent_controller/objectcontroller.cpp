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
#include "json/json.h"
#include "Config.h"
#include <iostream>
#include <string.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


ObjectController::ObjectController(const double period)//:RateThread(int(period*1000))
{
  Json::Value objectConfig = Config::instance()->root["object"];
  Json::Value trajConfig = objectConfig["trajectory"];
  
  // Initializing the private members
  m_VelX = 0.1; // 0.1 m/s
  m_Mean = trajConfig["noise"]["mean"].asDouble(); // Zero mean //TODO
  m_Sigma = trajConfig["noise"]["sigma"].asDouble(); // Standard Deviation in Position //TODO
  m_Period = period; // Period of the Thread;
  
  // Resizing the position vectors
  m_initPosition.resize(3);
  m_currPosition.resize(3);
  
  // Set the initial position. 
  // TODO Read from config file
  Json::ArrayIndex i=0;
  m_initPosition[0] = objectConfig["initialPos"].get(i++,Json::Value(0)).asDouble();
  m_initPosition[1] = objectConfig["initialPos"].get(i++,Json::Value(1)).asDouble();
  m_initPosition[2] = objectConfig["initialPos"].get(i++,Json::Value(0.45)).asDouble();
  std::cout << "Initial Position : " << m_initPosition.toString(-1,1) << std::endl;
  
  m_currPosition = m_initPosition;
    
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
  
  m_start.resize(3);
  m_end.resize(3);
  m_step.resize(3);
    
  m_currStep = 0;
  
  i=0;
  m_start[0]=trajConfig["linear"]["start"].get(i++,Json::Value(-10)).asDouble();
  m_start[1]=trajConfig["linear"]["start"].get(i++,Json::Value(53.39)).asDouble();
  m_start[2]=trajConfig["linear"]["start"].get(i++,Json::Value(35)).asDouble();
  
  std::cout << "Start : " << m_start.toString(-1,1) << std::endl;
  
  i=0;
  m_end[0]=trajConfig["linear"]["end"].get(i++,Json::Value(8)).asDouble();
  m_end[1]=trajConfig["linear"]["end"].get(i++,Json::Value(53.39)).asDouble();
  m_end[2]=trajConfig["linear"]["end"].get(i++,Json::Value(35)).asDouble();
  
  std::cout << "End : " << m_end.toString(-1,1) << std::endl;

  m_numPoints=objectConfig["trajectory"]["samples"].asInt();
  if(m_numPoints==0){
    m_numPoints=1;
  }
  
  m_radius=objectConfig["radius"].asDouble();
  if(m_radius > 1){
    m_radius/=100; // cm->m
  }else{
    m_radius = 0.03; //default
  }
  
  if(m_numPoints==1){
    m_static = true;
    for(int i=0;i<3;i++){
      m_step[i]=0;
    }
  }else{
    m_static = false;
    for(int i=0;i<3;i++){
      m_step[i]=(m_end[i]-m_start[i])/(m_numPoints-1);
    }
  }
  
  m_multiple=50;
  m_currmult=0;
  m_stop = true;
#if MYBOX
  radius=20;
  xc=0;
  yc=65;
  zc=40;

  ballPos[0] = 0;
  ballPos[1] = boxPos[1]+boxSize[1]/2+ball_radius;
  ballPos[2] = 0.25;
#else
  
  radius=10;
  xc=0;
  yc=53.3951;
  zc=40;

//   10 54.39 35
  ballPos[0] = m_start[0]/100;
  ballPos[1] = m_start[1]/100;
  ballPos[2] = m_start[2]/100;
//   ballPos = m_start;
#endif
  m_currPosition = ballPos;
}

ObjectController::~ObjectController()
{

}

bool ObjectController::open(yarp::os::ResourceFinder &rf){
  bool ret=true;   
  ret = objectCmdPort.open("/object/cmd/in");
  ret &= objectStatusPort.open("/object/status/out");    
    
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
  
  std::cout << "ObjectCtrl Thread started successfully\n";
  Bottle del_all("world del all"); 
  outputStatePort.write(del_all,reply); 
  
  std::string str;
#if BOX
  str = "world mk sbox ";
  str = str + boxSize.toString(-1,1).c_str();
  str = str + " " + boxPos.toString(-1,1).c_str();
  str = str + " 0.8 0.8 0.8";
  std::cout << str << std::endl;
//     Bottle create_box("world mk sbox 1 0.1 0.5 0.0 0.4 0.6 0.8 0.8 0.8");
  Bottle create_box(ConstString(str.c_str()));
  outputStatePort.write(create_box,reply);
#endif
  
  if(!m_static)
    str = "world mk sph ";
  else
    str = "world mk ssph ";
  Vector rad;rad.push_back(m_radius/2);
  str = str + string(rad.toString().c_str());
  str = str + " " + m_currPosition.toString(-1,1).c_str();
  str = str + " 0 1 0";
  std::cout << str << std::endl;
  Bottle create_ball(ConstString(str.c_str()));
  outputStatePort.write(create_ball,reply);
  
  startTime = Time::now(); 
    
  return ret;
}

bool ObjectController::close(){
  outputStatePort.close();
}

void ObjectController::loop(){
  
  // Read command from Planner
  Bottle* cmd = objectCmdPort.read(false);
  if(cmd)
  {
    double command = cmd->get(0).asDouble();
    std::cout << "Received something from Planner: " << command << std::endl;
    if(command > 0){
      if(m_stop){
	m_stop = false;
	std::cout << "Object motion Started from: " << m_currPosition.toString() << std::endl;
      }
    }else{
      if(!m_stop){
	m_stop = true;
	std::cout << "Object motion Stopped at: " << m_currPosition.toString() << std::endl;
      }
    }
  }
  
  // Send ball position to iCub World
  m_currmult++;
  if(m_currmult==m_multiple)
    m_currmult=0;
  if(!(m_currmult%m_multiple==0)){
    //return;
  }else{
    if(!m_stop){
      string str;
    if(m_static)
      str = "world set ssph 1";
    else
      str = "world set sph 1";
    
    Bottle move_obj(str); 
      
    #if 0
      yarp::sig::Vector vec = m_currPosition;
    #else
      yarp::sig::Vector vec = getNextPos();	
    #endif
      move_obj.addDouble(vec[0]); 
      move_obj.addDouble(vec[1]); 
      move_obj.addDouble(vec[2]);
      outputStatePort.write(move_obj,reply); 
    }
  }
  
  // Send ball status to Planner
  Bottle &status = objectStatusPort.prepare();
  status.clear();
  status.addDouble(m_currPosition[0]);
  status.addDouble(m_currPosition[1]);
  status.addDouble(m_currPosition[2]);
  objectStatusPort.write();
}

bool ObjectController::interrupt(){
  return true;
}

yarp::sig::Vector ObjectController::getNextPosition(){
  double t = Time::now() - startTime;   
#if MYBOX
  double dx = (round(xc + radius*cos(t))/100); 
  double dy = (round(yc + radius*sin(t))/100); 
  double dz = round(zc)/100;
#else
  double dx = (round(xc + radius*cos(t))/100); 
  double dz = (round(zc + radius*sin(t))/100); 
  double dy = round(yc)/100;
#endif
  m_currPosition[0] = dx;
  m_currPosition[1] = dy;
  m_currPosition[2] = dz;
  return m_currPosition;
}

yarp::sig::Vector ObjectController::getNextPos(){
  if(m_currStep==m_numPoints){
    m_currStep=0;
  }
  double dx = (m_start[0]+m_step[0]*m_currStep)/100;
  double dy = (m_start[1]+m_step[1]*m_currStep)/100;
  double dz = (m_start[2]+m_step[2]*m_currStep)/100;
  m_currPosition[0] = dx;
  m_currPosition[1] = dy;
  m_currPosition[2] = dz;
  m_currStep++;
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