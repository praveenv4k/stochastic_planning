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
#include "Trajectory.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


ObjectController::ObjectController(const double period)//:RateThread(int(period*1000))
{
  Json::Value objectConfig = Config::instance()->root["object"];
  Json::Value trajConfig = objectConfig["trajectory"];
  Json::Value elbowConfig = Config::instance()->root["elbow"];
  
  // Initializing the private members
  m_VelX = 0.1; // 0.1 m/s
  m_Mean = 0; // Zero mean 
  m_Sigma = 0; // Standard Deviation in Position 
  m_Period = period; // Period of the Thread;
  
  // Resizing the position vectors
  m_initPosition.resize(3);
  m_currPosition.resize(3);
  m_exactPosition.resize(3);
  
  // Set the initial position. 
  // TODO Read from config file
  Json::ArrayIndex i=0;
  m_initPosition[0] = objectConfig["initialPos"].get(i++,Json::Value(0)).asDouble();
  m_initPosition[1] = objectConfig["initialPos"].get(i++,Json::Value(1)).asDouble();
  m_initPosition[2] = objectConfig["initialPos"].get(i++,Json::Value(0.45)).asDouble();
  std::cout << "Initial Position : " << m_initPosition << std::endl;
  
  // Y- UP
  // X - Left/Right of Robot
  // Z - In front of the robot
  //Vector boxSize;

  ball_radius = 0.02;
      
  m_currStep = 0;
  
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
  }else{
    m_static = false;
  }
  
  m_noiseEnabled=false;
  if(!trajConfig["noise"].isNull()){
    m_noiseEnabled=trajConfig["noise"].asBool();
    if(m_noiseEnabled){
      m_Mean = trajConfig["noise"]["mean"].asDouble();
      m_Sigma = trajConfig["noise"]["sigma"].asDouble();
    }
  }
  
  m_elbowEnabled=false;
  TrajectoryDiscretizerPtr trajPtr = TrajectoryDiscretizerFactory::getTrajectoryDiscretizer(trajConfig);
  if(trajPtr!=NULL){
    if(trajPtr->getAllPoses(m_numPoints,m_objPoses)){
      
      if(m_noiseEnabled){
	TrajectoryDiscretizer::getNoisyPoses(m_Mean,m_Sigma,m_objPoses,m_noisyObjPoses);
      }else{
	for(size_t i=0;i<m_objPoses.size();i++){
	  m_noisyObjPoses.push_back(m_objPoses[i]);
	}
      }
      
      if(!elbowConfig.isNull()){
	m_elbowEnabled = elbowConfig["enabled"].asBool();
	if(m_elbowEnabled){
	  std::vector<double> range;
	  if(Utils::valueToVector(elbowConfig["range"],range)){
	    double length = elbowConfig["length"].isNull() ? 30: elbowConfig["length"].asDouble();
	    m_elbowRadius = elbowConfig["radius"].asDouble();
	    m_elbowRadius/=100;
	    std::cout << "Get Elbow pos: " << length << "," << range[0] << "," << range[1] << std::endl;
	    TrajectoryDiscretizer::getElbowPoses(m_noisyObjPoses,m_elbowPoses,range[0],range[1],length);
	  }
	}
      }
    }
  }
  
   
  m_multiple=20;
  m_currmult=0;
  m_stop = true;

  if(m_noisyObjPoses.size()>0){
    m_currPosition = m_noisyObjPoses[0]/100;
    m_exactPosition = m_objPoses[0]/100;
  }else{
    m_currPosition = m_initPosition/100;
    m_exactPosition = m_initPosition/100;
  }
  
  if(m_elbowPoses.size()>0){
    m_currElbowPosition = m_elbowPoses[0]/100;
  }
  
  //m_stop = false;
  
  std::cout << "Object constructed" << std::endl;
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
  Bottle create_box(ConstString(str.c_str()));
  outputStatePort.write(create_box,reply);
#endif
  
  if(!m_static)
    str = "world mk sph ";
  else
    str = "world mk ssph ";
  Vector rad;
  rad.push_back(m_radius); //TODO
  str = str + string(rad.toString().c_str());
  stringstream ss;
  ss << m_currPosition;
  str = str + " " + ss.str();
  str = str + " 0 1 0";
  std::cout << str << std::endl;
  Bottle create_ball(ConstString(str.c_str()));
  outputStatePort.write(create_ball,reply);
  
  if(m_elbowEnabled && m_elbowPoses.size()>0){
    str = "world mk ssph ";
    Vector elb;
    elb.push_back(m_elbowRadius);
    str = str + string(elb.toString().c_str());
    stringstream ss;
    ss << m_currElbowPosition;
    str = str + " " + ss.str();
    str = str + " 0 0 1";
    std::cout << str << std::endl;
    Bottle create_elbow(ConstString(str.c_str()));
    outputStatePort.write(create_elbow,reply);
  }
  
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
	std::cout << "Object motion Started from: " << m_currPosition << std::endl;
      }
    }else{
      if(!m_stop){
	m_stop = true;
	std::cout << "Object motion Stopped at: " << m_currPosition << std::endl;
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
      Container<double> vec = getNextPosition();	
    #endif
      move_obj.addDouble(vec[0]); 
      move_obj.addDouble(vec[1]); 
      move_obj.addDouble(vec[2]);
      outputStatePort.write(move_obj,reply); 
      //cout << move_obj.toString() << std::endl;
      if(m_elbowEnabled && m_elbowPoses.size()>0){
	string str;
	str = "world set ssph 1";
	
	Bottle move_elb(str); 
	move_elb.addDouble(m_currElbowPosition[0]); 
	move_elb.addDouble(m_currElbowPosition[1]); 
	move_elb.addDouble(m_currElbowPosition[2]);
	outputStatePort.write(move_elb,reply); 
	//cout << move_elb.toString() << std::endl;
      }
    }
  }
  
  // Send ball status to Planner
  Bottle &status = objectStatusPort.prepare();
  status.clear();
  status.addDouble(m_exactPosition[0]);
  status.addDouble(m_exactPosition[1]);
  status.addDouble(m_exactPosition[2]);
  status.addDouble(m_currPosition[0]);
  status.addDouble(m_currPosition[1]);
  status.addDouble(m_currPosition[2]);
  objectStatusPort.write();
}

bool ObjectController::interrupt(){
  return true;
}

Container<double> ObjectController::getNextPosition(){
  if(m_currStep==m_numPoints || m_currStep>=m_objPoses.size()){
    m_currStep=0;
  }
  
//   double dx = m_objPoses[m_currStep][0]/100;
//   double dy = m_objPoses[m_currStep][1]/100;
//   double dz = m_objPoses[m_currStep][2]/100;
//   m_currPosition[0] = dx;
//   m_currPosition[1] = dy;
//   m_currPosition[2] = dz;
  
  m_currPosition = m_noisyObjPoses[m_currStep]/100;
  m_exactPosition = m_objPoses[m_currStep]/100;
  if(m_elbowEnabled && m_elbowPoses.size()>0){
    m_currElbowPosition = m_elbowPoses[m_currStep]/100;
  }
  
  m_currStep++;
  return m_currPosition;
}
