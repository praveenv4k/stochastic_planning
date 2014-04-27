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


#ifndef ARMCONTROLLER_H
#define ARMCONTROLLER_H

/**
 * @file ArmCtrlThread.cpp
 * This thread looks to provide a control strategy for arm movement.
 * @brief Arm control thread
 *
 */

#include <Configuration.h>
#include <StateAction.h>
//#include <Exploration.h>


struct ControllerState
{
    enum pose { beg, tap, push_right, invalid};

    yarp::sig::Vector posn;
    yarp::sig::Vector ortn;
    std::vector<yarp::sig::Vector > allOrients;
    double time;

    ControllerState()
    {
        using namespace yarp::math;

        posn.resize(3);
        ortn.resize(4);     /* in case of real robot, this is axis-rot notation */
        allOrients.resize(3);   /* we consider beg, tap and push right only */
        yarp::sig::Vector oy(4), ox(4);
        yarp::sig::Matrix Ry, Rx, R;

        /* Try beg */
        oy[0]=0.0; oy[1]=0.0; oy[2]=1.0; oy[3]=M_PI;
        R= iCub::ctrl::axis2dcm(oy);   // from axis/angle to rotation matrix notation
        allOrients[beg] = iCub::ctrl::dcm2axis(R);


        /* Tap */
        oy[0]=0.0; oy[1]=1.0; oy[2]=0.0; oy[3]=M_PI;
        R= iCub::ctrl::axis2dcm(oy);   // from axis/angle to rotation matrix notation
        allOrients[tap] = iCub::ctrl::dcm2axis(R);

        /* Push right */
        oy[0]=0.0; oy[1]=1.0; oy[2]=0.0; oy[3]=M_PI;
        ox[0]=1.0; ox[1]=0.0; ox[2]=0.0; ox[3]=M_PI/2.0;
        Ry= iCub::ctrl::axis2dcm(oy);   // from axis/angle to rotation matrix notation
        Rx= iCub::ctrl::axis2dcm(ox);
        R=Ry*Rx;
        allOrients[push_right] = iCub::ctrl::dcm2axis(R);
    }

    ControllerState(ControllerState &cs)
    {
        posn.resize(3);
        ortn.resize(4);

        posn[0] = cs.posn[0];posn[1] = cs.posn[1];posn[2] = cs.posn[2];
        ortn[0] = cs.ortn[0];ortn[1] = cs.ortn[1];
        ortn[2] = cs.ortn[2];ortn[3] = cs.ortn[3];
    }

    void toLearnerState(State& s)
    {
        s.SetState(posn,double(getCurrentOrientTag()));
    }

    void fromLearnerState(State& s)
    {
        posn[0] = s.disc[0];posn[1] = s.disc[1];posn[2] = s.disc[2];
        ortn = allOrients[s.disc[3]];
    }

    int getCurrentOrientTag()
    {
        if(fabs(allOrients[beg][0] - ortn[0]) < EPSILON &&
                fabs(allOrients[beg][1] - ortn[1]) < EPSILON &&
                fabs(allOrients[beg][2] - ortn[2]) < EPSILON &&
                fabs(allOrients[beg][3] - ortn[3]) < EPSILON)
            return beg;

        if(fabs(allOrients[tap][0] - ortn[0]) < EPSILON &&
                fabs(allOrients[tap][1] - ortn[1]) < EPSILON &&
                fabs(allOrients[tap][2] - ortn[2]) < EPSILON &&
                fabs(allOrients[tap][3] - ortn[3]) < EPSILON)
            return tap;

        if(fabs(allOrients[push_right][0] - ortn[0]) < EPSILON &&
                        fabs(allOrients[push_right][1] - ortn[1]) < EPSILON &&
                        fabs(allOrients[push_right][2] - ortn[2]) < EPSILON &&
                        fabs(allOrients[push_right][3] - ortn[3]) < EPSILON)
                    return push_right;

        return invalid;

    }

};

class CtrlThread : public yarp::os::RateThread
                  //,public CartesianEvent
{
protected:

    yarp::os::BufferedPort<yarp::os::Bottle > outputSigPort;     // sends a bottle with 2 bools: action finished, reset state
    yarp::os::BufferedPort<yarp::os::Bottle > outputStatePort;     // sends a bottle with 2 bools: action finished, reset state
    yarp::os::Port detectCollisionPort;
    yarp::os::BufferedPort<yarp::os::Bottle > inputPort;

    bool m_collision;       // is aware of obstacle or not
    bool m_internalCollision;
    double m_delayAfterAct;
    std::string m_name;
    yarp::sig::Vector initQd, nowQd;


    ControllerState nowCS, previousCS, resetCS;
    State nowLS, previousLS, resetLS;

    yarp::dev::PolyDriver         ddCart;
    yarp::dev::ICartesianControl *icart;

    yarp::dev::PolyDriver       ddArm;
    yarp::dev::IPositionControl *posCtrlArm;

    yarp::dev::PolyDriver       ddTorso;
    yarp::dev::IPositionControl *posCtrlTorso;

public:

    //bool m_shouldMove;      /* Flag to set arm to motion */

    CtrlThread() : RateThread(10)
    {
        m_name = "MobeeReach";
    }
    ~CtrlThread()
    {

    }

    bool threadInit();

    void afterStart(bool s);

    void run();


    bool detectCollision();

    void reset();

    void threadRelease();


    inline void GetCurrentState(State& st)
    {
        st = nowLS;
    }

    inline void setReset(State& st)
    {
        resetCS.fromLearnerState(st);
    }

    inline void setName(std::string val)
    {
        m_name = val;
    }

    inline std::string getName()
    {
        return m_name;
    }

    inline void moveTo(ControllerState& cs)
    {
        if(Config::instance()->root["Robot"]["Filtered"].asBool())
        {
            yarp::sig::Vector tmpQ, tmpX, tmpO;
            tmpQ.resize(10);
            tmpX.resize(3);
            tmpO.resize(4);
            const yarp::sig::Vector postn = cs.posn;
            const yarp::sig::Vector orntn = cs.ortn;
            if(! icart->askForPosition(nowQd,postn,tmpX,tmpO,tmpQ))
                std::cout << " Inversion failed!!!\n ";


            std::cout << " Went from qd = " << nowQd.toString()
                      << " to qd = " << tmpQ.toString() << "\n";
            std::cout << "For commanded pos: " << cs.posn.toString()
                      <<" and orientation: " << cs.ortn.toString() << "\n";

            moveJoints(tmpQ);

            // Update
            cs.posn = tmpX;
            cs.ortn = tmpO;
            nowQd = tmpQ;
        }
        else
        {
            icart->goToPoseSync(cs.posn,cs.ortn,cs.time);
            bool done=false;
            while (!done) {
               icart->checkMotionDone(&done);
               yarp::os::Time::delay(0.04);   // or any suitable delay
            }
            yarp::os::Time::delay(m_delayAfterAct);
        }

        getCtrlState(cs);

        //yarp::os::Time::delay(1);
    }

    inline void resetJoints()
    {
        Json::Value armJnts = Config::instance()->root["Parts"]["right_arm"]["pose_init"];
        Json::Value torsoJnts = Config::instance()->root["Parts"]["torso"]["pose_init"];

        bool done = false;
        yarp::sig::Vector torsoPos;
        torsoPos.resize(3);
        for(Json::ArrayIndex i=0 ; i < 3; ++i)
        {
            torsoPos[i] = torsoJnts[i].asDouble();
            nowQd[i] = torsoJnts[i].asDouble();
        }
        
        posCtrlTorso->positionMove(torsoPos.data());
        
	do
        {
           posCtrlTorso->checkMotionDone(&done);
           yarp::os::Time::delay(.01);
        }while(!done);
	
        yarp::sig::Vector armPos;
        armPos.resize(16);
        
	for(Json::ArrayIndex i=0; i < 16; ++i)
        {
            armPos[i] = armJnts[i].asDouble();
            if(i<7)
                nowQd[i+3] = armJnts[i].asDouble();
        }
        
        posCtrlArm->positionMove(armPos.data());
        
	do
        {
           posCtrlArm->checkMotionDone(&done);
           yarp::os::Time::delay(.01);
        }while(!done);
	
        std::cout << "Reset to pos: "
                  << torsoPos.toString() << " and arm: "
                  << armPos.toString() << "\n";
    }
    
    inline void moveJoints(yarp::sig::Vector &qd)
    {
        bool done = false;
        yarp::sig::Vector torsoPos;
        torsoPos.resize(3);
        for(int i=0 ; i < 3; ++i)
        {
            torsoPos[i] = qd[i];
        }
        posCtrlTorso->positionMove(torsoPos.data());
        do
        {
           posCtrlTorso->checkMotionDone(&done);
           yarp::os::Time::delay(.01);
        }while(!done);

        yarp::sig::Vector armPos;
        armPos.resize(16);
        
	for(int i=0; i < 7; ++i)
        {
            armPos[i] = qd[i+3];
        }
        
        for(Json::ArrayIndex i=7; i<16; ++i)
        {
            armPos[i] = Config::instance()->root["Parts"]["right_arm"]["pose_init"][i].asDouble();
        }
        
        posCtrlArm->positionMove(armPos.data());
        
	do
        {
           posCtrlArm->checkMotionDone(&done);
           yarp::os::Time::delay(.01);
        }while(!done);
	
        std::cout << "Move to pos: "
                  << torsoPos.toString() << " and arm: "
                  << armPos.toString() << "\n";
    }

    inline void getCtrlState(ControllerState &cs)
    {
        yarp::sig::Vector dumb; /* Neglect orientation */
        icart->getPose(cs.posn,dumb);
    }

};

#endif // ARMCONTROLLER_H
