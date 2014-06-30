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


#ifndef OBJECTCONTROLLER_H
#define OBJECTCONTROLLER_H

#include "Global.h"

using namespace yarp::os;

class ObjectController
{
public:
    //yarp::os::BufferedPort<yarp::os::Bottle > outputStatePort; 
    
//     yarp::os::RpcClient outputStatePort;
    Port outputStatePort;
    
    ObjectController(const double period);
    virtual ~ObjectController();
    std::string getName() const{
      return "objectCtrl";
    }
    yarp::sig::Vector getNextPosition();
    yarp::sig::Vector getNextPos();
    std::string getPositionStr(yarp::sig::Vector& vector) const;
    
//     bool threadInit();
//     void afterStart(bool s);
//     void run();
//     void reset();
//     void threadRelease();
    
    bool open(yarp::os::ResourceFinder &rf);
    bool close();
    void loop(); 
    bool interrupt();
private:
    ObjectController(const ObjectController& other)//:RateThread(10)
    {
    }
    virtual ObjectController& operator=(const ObjectController& other){
      return *this;
    }
    virtual bool operator==(const ObjectController& other) const{
    }
    
    yarp::sig::Vector m_currPosition;
    yarp::sig::Vector m_initPosition;
    
    yarp::os::Bottle outputState;
    yarp::os::Bottle reply;
    
    yarp::os::Network yarp; 
    
    double m_VelX;
    double m_Mean;
    double m_Sigma;
    int m_Period;
    
    double startTime;
    double xc,yc,zc,radius;
    
    yarp::sig::Vector boxPos;
    yarp::sig::Vector boxSize;
    yarp::sig::Vector ballPos;
    
    yarp::sig::Vector m_start;
    yarp::sig::Vector m_end;       
    int m_currStep;
    yarp::sig::Vector m_step;
 
    double ball_radius;
    int m_multiple;
    int m_currmult;
    
    bool m_stop;
    bool m_static;
    
    double m_radius;
    int m_numPoints;
    
    yarp::os::BufferedPort<yarp::os::Bottle> objectCmdPort;
    yarp::os::BufferedPort<yarp::os::Bottle> objectStatusPort;
};

#endif // OBJECTCONTROLLER_H
