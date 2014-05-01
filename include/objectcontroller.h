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

class ObjectController:public yarp::os::RateThread
{
public:
    yarp::os::BufferedPort<yarp::os::Bottle > outputStatePort; 
    
    ObjectController(int period);
    virtual ~ObjectController();
    std::string getName() const{
      return "objectCtrl";
    }
    yarp::sig::Vector getNextPosition();
    std::string getPositionStr(yarp::sig::Vector& vector) const;
    
    bool threadInit();
    void afterStart(bool s);
    void run();
    void reset();
    void threadRelease();
    
private:
    ObjectController(const ObjectController& other):RateThread(10){
    }
    virtual ObjectController& operator=(const ObjectController& other){
      return *this;
    }
    virtual bool operator==(const ObjectController& other) const{
    }
    
    yarp::sig::Vector m_currPosition;
    yarp::sig::Vector m_initPosition;
    double m_VelX;
    double m_Mean;
    double m_Sigma;
    int m_Period;
};

#endif // OBJECTCONTROLLER_H
