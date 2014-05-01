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


#include "armcontroller.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

bool CtrlThread::threadInit()
{
    // Set collision
    //m_collision = Config::instance()->root["Robot"]["Collision"].asBool();
    //m_internalCollision = Config::instance()->root["Robot"]["InternalCollision"].asBool();
    //m_delayAfterAct = Config::instance()->root["Robot"]["DelayAfterAct"].asDouble();
    //m_shouldMove = false;

    // Record init pose
    Json::Value armJnts = Config::instance()->root["Parts"]["right_arm"]["pose_init"];
    Json::Value torsoJnts = Config::instance()->root["Parts"]["torso"]["pose_init"];
    initQd.resize(10);
    nowQd.resize(10);
    for(Json::ArrayIndex i=0; i<3; ++i)
    {
        initQd[i] = torsoJnts[i].asDouble();
    }
    for(Json::ArrayIndex i=3; i < 10; ++i)
    {
        initQd[i] = armJnts[(i-3)].asDouble();
    }
    nowQd = initQd;


    string outputPortName = "/" ;
    outputPortName += getName();
    outputPortName += "/flags:o";

    if (!outputSigPort.open(outputPortName.c_str()))
    {
        std::cout <<": unable to open port to send flags\n";
        return false;  // unable to open; let RFModule know so that it won't run

    }

    outputPortName = "/" ;
    outputPortName += getName();
    outputPortName += "/states:o";
    if (!outputStatePort.open(outputPortName.c_str()))
    {
        std::cout <<": unable to open port to send states\n";
        return false;  // unable to open; let RFModule know so that it won't run

    }

    string inputPortName = "/" ;
    inputPortName += getName();
    inputPortName += "/actions:i";

    if (!inputPort.open(inputPortName.c_str()))
    {
        std::cout <<": unable to open port to receive actions\n";
        return false;  // unable to open; let RFModule know so that it won't run

    }

    string collisonPortName = "/" ;
    collisonPortName += getName();
    collisonPortName += "/collision:i";

    if (!detectCollisionPort.open(collisonPortName.c_str()))
    {
        std::cout <<": unable to open port to MoBeE.\n";
        return false;  // unable to open; let RFModule know so that it won't run

    }

    /* Connect to MoBeE port */

    if(! Network::connect("/MoBeE/collisions",collisonPortName.c_str()))
    {
        std::cout <<": unable to connect to collision port of MoBeE.\n";
        return false;  // unable to open; let RFModule know so that it won't run

    }


    Json::Value initPos = Config::instance()->root["Position"]["Initial"];

    resetCS.posn[0] = initPos["X"].asDouble();
    resetCS.posn[1] = initPos["Y"].asDouble();
    resetCS.posn[2] = initPos["Z"].asDouble();

    /*This changes for iCub */

    resetCS.ortn = resetCS.allOrients[initPos["O"].asInt()];

    resetCS.time = 1; /* Assuming constant time for resetting */

    resetCS.toLearnerState(resetLS);


    // Both previous and current state are set to reset
    nowCS = resetCS;
    nowLS = resetLS;

    previousCS = resetCS;
    previousLS = resetLS;

    // Initialize cartesian control
    string remotePortName = "/";
    remotePortName += Config::instance()->root["Robot"]["Name"].asString();
    remotePortName += "/cartesianController/right_arm";
    Property option("(device cartesiancontrollerclient)");
    option.put("remote",remotePortName.c_str());
    option.put("local","/cartesian_client/right_arm");

    if (!ddCart.open(option))
        return false;

    // open the view
    ddCart.view(icart);

    // set trajectory time
    icart->setTrajTime(1.0);

    // get the torso dofs
    Vector newDof, curDof;
    icart->getDOF(curDof);
    newDof=curDof;

    // enable the torso yaw and pitch
    // disable the torso roll
    newDof[0]=1;
    newDof[1]=1;
    newDof[2]=1;

    // send the request for dofs reconfiguration
    icart->setDOF(newDof,curDof);

    // Dont bend more than 15 degrees
    double min, max;
    icart->getLimits(0,&min,&max);
    icart->setLimits(0,min,15);

    // Dont turn more than 15
    icart->setLimits(2,-15,15);
    icart->setLimits(1,-15,15);


    // print out some info about the controller
    Bottle info;
    icart->getInfo(info);
    cout << "info = " << info.toString() << "\n";
    cout << "DOFs = " << curDof.toString() << " and new: "
         << newDof.toString() << " \n";


    /* Initialize position control for arm and torso */
    /* Right Arm */
    remotePortName = "/";
    remotePortName += Config::instance()->root["Robot"]["Name"].asString();
    if(Config::instance()->root["Robot"]["Filtered"].asBool())
    {
        remotePortName += "F";
    }
    remotePortName += "/right_arm";
    Property optionArm("(device remote_controlboard)");
    optionArm.put("remote",remotePortName.c_str());
    optionArm.put("local","/mobeeReach/F/right_arm");

    if (!ddArm.open(optionArm))
        return false;

    // open the view
    ddArm.view(posCtrlArm);

    /* Torso */
    remotePortName = "/";
    remotePortName += Config::instance()->root["Robot"]["Name"].asString();
    if(Config::instance()->root["Robot"]["Filtered"].asBool())
    {
        remotePortName += "F";
    }
    remotePortName += "/torso";
    Property optionTorso("(device remote_controlboard)");
    optionTorso.put("remote",remotePortName.c_str());
    optionTorso.put("local","/mobeeReach/F/torso");

    if (!ddTorso.open(optionTorso))
        return false;

    // open the view
    ddTorso.view(posCtrlTorso);

    //moveTo(resetCS);
    resetJoints();

    return true;
}

void CtrlThread::threadRelease()
{
    outputSigPort.close();
    outputStatePort.close();
    detectCollisionPort.close();

}

void CtrlThread::afterStart(bool s)
{
    if (s)
        std::cout << "Thread started successfully\n";
    else
        std::cout << "Thread did not start\n";

}

void CtrlThread::run() //Action &act, State &nxtState, bool &reached, bool &invalidAct)
{
    yarp::os::Bottle* actionRead;
    actionRead = inputPort.read(false);
    bool invalidAct, reached;

    if(actionRead)
    {
        if(actionRead->size() == 4) /* RESET */
        {
            nowCS.posn[0] = actionRead->get(0).asDouble();
            nowCS.posn[1] = actionRead->get(1).asDouble();
            nowCS.posn[2] = actionRead->get(2).asDouble();
            /* neglect orientation */
            resetCS = nowCS;
            reset();
        }
        else
        {

            previousCS = nowCS;
            previousCS.toLearnerState(previousLS);

            nowCS.posn[0] += actionRead->get(0).asDouble();
            nowCS.posn[1] += actionRead->get(1).asDouble();
            nowCS.posn[2] += actionRead->get(2).asDouble();

            int ort = actionRead->get(3).asInt();

            nowCS.time = actionRead->get(4).asDouble();

            /* Orientation */
            int newOrt = nowCS.getCurrentOrientTag() + ort;
            if(newOrt = nowCS.invalid)
            {
                invalidAct = true;
                newOrt = 0;
            }
            else
                invalidAct = false;
            nowCS.ortn = nowCS.allOrients[newOrt];

            moveTo(nowCS);

            nowCS.toLearnerState(nowLS);

            reached = nowLS.ReachedGoal();
            invalidAct = invalidAct || nowLS.IsInvalidState();

            // Does it collide
            //Config::instance()->hasCollided = m_collision &&
            //        detectCollision();


            if(/*Config::instance()->hasCollided ||*/ invalidAct)
            {
                nowLS.Invalidate();

                reset();
            }
        }




    }

    Bottle& outputState = outputStatePort.prepare();
    outputState.clear();
    outputState.addDouble(nowLS.disc[0]);
    outputState.addDouble(nowLS.disc[1]);
    outputState.addDouble(nowLS.disc[2]);
    outputState.addDouble(nowLS.disc[3]);
    outputStatePort.writeStrict();

    Bottle& outputFlags = outputSigPort.prepare();
    outputFlags.addInt(invalidAct);
    outputFlags.addInt(reached);
    outputSigPort.writeStrict();




}

void CtrlThread::reset()
{
    nowCS = resetCS;
    resetJoints();
    moveTo(nowCS);
    nowCS.toLearnerState(nowLS);
    //m_shouldMove = false;

    yarp::os::Time::delay(0.2);
}

// bool CtrlThread::detectCollision()
// {
//     if(m_collision)
//     {
//         Bottle fromCollisionPort;
//         fromCollisionPort.clear();
//         detectCollisionPort.read(fromCollisionPort);
// 
//         int sz = fromCollisionPort.size();
// 
//         if(sz > 0 &&
//                 !(sz ==1 && fromCollisionPort.get(0).asInt() == 0 ))
//         {
//             if(m_internalCollision)
//                 return true;
//             else
//             {
//                 for(int i=0; i<sz; ++i)
//                 {
// 
//     //                    cout << "Received Bottle of size " <<
//     //                            sz << " with " << i <<"th elem as: " <<
//     //                        fromCollisionPort.get(i).asList()->toString() << " \n";
//                     if(fromCollisionPort.get(i).asList()->size() == 5)
//                     {
//                         std::cout << "Collision detected. Press a key\n";
//                         std::cin >> sz;
//                         return true;
// 
//                     }
//                 }
//                 return false;
//             }
// 
//         }
// 
//     }
// 
//     return false;
// }









