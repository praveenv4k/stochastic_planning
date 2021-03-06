#ifndef __TRAJECTORY_H__
#define __TRAJECTORY_H__

#define _USE_MATH_DEFINES
#include "Container.h"
#include <cmath>
#include <boost/shared_ptr.hpp>
#include <boost/math/constants/constants.hpp>

class TrajectoryDiscretizer;
class Trajectory;

typedef boost::shared_ptr<TrajectoryDiscretizer> TrajectoryDiscretizerPtr;
typedef boost::shared_ptr<Trajectory> TrajectoryPtr;

class TrajectoryDiscretizer{
public:
  TrajectoryDiscretizer(){
  }
  virtual bool getNextPose(double stepSize, double currentStep,Container<double>& pose)=0;
  virtual bool getAllPoses(int numPoints, std::vector<Container<double> >& poses)=0;
};

class BernoulliDiscretizer:public TrajectoryDiscretizer{
public:
  BernoulliDiscretizer(double focalDistance):m_focalDistance(focalDistance){
  }
  virtual bool getNextPose(double stepSize, double currentStep,Container<double>& pose){
    pose.resize(3);
    double root2 = sqrt(2);
    double den = (sin(currentStep)*sin(currentStep))+1.0;
    double dx = m_focalDistance*root2*cos(currentStep)/den;
    double dy = m_focalDistance*root2*cos(currentStep)*sin(currentStep)/den;
    
    pose[0] = (dx); 
    pose[1] = (dy); 
    pose[2] = (0.65);
    
    return true;
  }
  
  virtual bool getAllPoses(int numPoints, std::vector<Container<double> >& poses){
    return false;
  }
private:
  double m_focalDistance;
};

class CircleTrajectoryDiscretizer:public TrajectoryDiscretizer{
public:
  CircleTrajectoryDiscretizer(double xc,double yc,double zc, double radius)
      :m_radius(radius*100),m_xc(xc*100),m_yc(yc*100),m_zc(zc*100){
  }
  virtual bool getNextPose(double stepSize, double currentStep,Container<double>& pose){
    pose.resize(3);
    pose[0] = (round(m_xc + m_radius*cos(currentStep))/100); 
    pose[1] = (round(m_yc + m_radius*sin(currentStep))/100); 
    pose[2] = round(m_zc)/100;
    
    return true;
  }
  virtual bool getAllPoses(int numPoints, std::vector<Container<double> >& poses){
    poses.resize(numPoints);
    double step = (2*M_PI)/numPoints;
    for(int i=0;i<numPoints;i++){
      Container<double> pose;
      pose.resize(3);
//       pose[0] = (round(m_xc + m_radius*cos(i*step))/100); 
//       pose[1] = (round(m_yc + m_radius*sin(i*step))/100); 
//       pose[2] = round(m_zc)/100;
      pose[0] = (round(m_xc + m_radius*cos(i*step))/100); 
      pose[1] = round(m_yc)/100; 
      pose[2] = (round(m_zc+ m_radius*sin(i*step))/100); 
      poses[i] = pose;
    }
    return true;
  }
private:
  double m_radius;
  double m_xc,m_yc,m_zc;
};

class LinearTrajectoryDiscretizer:public TrajectoryDiscretizer{
  
public:
  
  LinearTrajectoryDiscretizer(Container<double> start,Container<double> end)
      :m_start(start),m_end(end){
  }
  
  virtual bool getNextPose(double stepSize, double currentStep,Container<double>& pose){
    return true;
  }
  
  virtual bool getAllPoses(int numPoints, std::vector<Container<double> >& poses){
    poses.resize(numPoints);
    Container<double> step;
    if(numPoints > 1){
      step = (m_end-m_start)/(numPoints-1);
    }
    else{
      step.resize(3);
    }
    for(int i=0;i<numPoints;i++){
      Container<double> pose;
      pose.resize(3);
      pose = m_start+step*i; 
      poses[i] = pose;
    }
    return true;
  }
  
private:
  Container<double> m_start;
  Container<double> m_end;
};

class Circle2DTrajectoryDiscretizer:public TrajectoryDiscretizer{
public:
  Circle2DTrajectoryDiscretizer(double xc,double yc, double radius)
      :m_radius(radius*100),m_xc(xc*100),m_yc(yc*100){
  }
  virtual bool getNextPose(double stepSize, double currentStep,Container<double>& pose){
    pose.resize(2);
    pose[0] = (round(m_xc + m_radius*cos(currentStep))/100); 
    pose[1] = (round(m_yc + m_radius*sin(currentStep))/100);
    return true;
  }
  virtual bool getAllPoses(int numPoints, std::vector<Container<double> >& poses){
    poses.resize(numPoints);
    double step = (2*M_PI)/numPoints;
    for(int i=0;i<numPoints;i++){
      Container<double> pose;
      pose.resize(2);
      pose[0] = (round(m_xc + m_radius*cos(i*step))/100); 
      pose[1] = (round(m_yc + m_radius*sin(i*step))/100);
      poses[i] = pose;
    }
    return true;
  }
private:
  double m_radius;
  double m_xc,m_yc;
};

class Trajectory{
public:
  Trajectory(double step, TrajectoryDiscretizerPtr trajDiscPtr )
      :m_step(step),m_currentStep(0),m_trajDiscPtr(trajDiscPtr){ 
    if(m_trajDiscPtr!=NULL){
      if(m_trajDiscPtr->getNextPose(m_step,m_currentStep,m_init)){
	m_current = m_init;
      }
    }
  }
  void reset(){
    m_current = m_init;
  }
  Container<double> getNextPose(){
    //Container<double> delta = 
    if(m_trajDiscPtr!=NULL){
      if(m_trajDiscPtr->getNextPose(m_step,m_currentStep+m_step,m_current)){
	m_currentStep+=m_step;
      }
    }
    return m_current;
  }
  Container<double>& getInitPose(){
    return m_init;
  }
  Container<double>& getCurrentPose(){
    return m_current;
  }
  bool getAllPoses(int numPoints, std::vector<Container<double> >& poses){
    if(m_trajDiscPtr !=NULL){
      return m_trajDiscPtr->getAllPoses(numPoints, poses);
    }
    return false;
  }
private:
  Container<double> m_init;
  Container<double> m_current;
  double m_step;
  double m_currentStep;
  TrajectoryDiscretizerPtr m_trajDiscPtr;
};

#endif