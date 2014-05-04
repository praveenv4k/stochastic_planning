#ifndef __TRAJECTORY_H__
#define __TRAJECTORY_H__

#include "Container.h"
#include <math.h>
#include <boost/shared_ptr.hpp>


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
    
    //dx = m_focalDistance* cos(currentStep);
    //dy = m_focalDistance* sin(currentStep);
    
    //pose[0] = (dx/8); 
    //pose[1] = (dy/8+0.75); 
    //pose[2] = (0.65);
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
    double step = M_PI/numPoints;
    for(int i=0;i<numPoints;i++){
      Container<double> pose;
      pose.resize(3);
      pose[0] = (round(m_xc + m_radius*cos(i*step))/100); 
      pose[1] = (round(m_yc + m_radius*sin(i*step))/100); 
      pose[2] = round(m_zc)/100;
      poses[i] = pose;
    }
    return true;
  }
private:
  double m_radius;
  double m_xc,m_yc,m_zc;
};

typedef boost::shared_ptr<TrajectoryDiscretizer> TrajectoryDiscretizerPtr;

class Trajectory{
public:
//   Trajectory(Container<double> init, double step, TrajectoryDiscretizerPtr trajDiscPtr )
//       :m_init(init),m_current(init),m_step(step),m_currentStep(0),m_trajDiscPtr(trajDiscPtr){ 
//     if(m_trajDiscPtr!=NULL){
//       if(m_trajDiscPtr->getNextPose(m_step,0,m_current)){
//       }
//     }
//   }
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