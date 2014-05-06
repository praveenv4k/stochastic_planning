#ifndef __UTILS_H__
#define __UTILS_H__

#include <stdexcept>
#include <math.h>
#include "json/json.h"

#define EPSILON 1e-6

//Part of the functions in the Utils class are taken from 
//https://bitbucket.org/kritisen/utilitiescpp
class Utils{
public:
  static void ind2sub(std::vector<int> siz, int idx, std::vector<int>& sub){
    size_t N = siz.size();
    int *prod = new int [N];
    for (size_t i = 0; i < N; i++){
      prod[i] = 1;
	  for (size_t j = N - 1; j > i; j--){
	prod[i] *= siz[j];	
      }
    }
    sub.resize(N);
	for (size_t i = 0; i < N; i++){
      sub[i] = idx ;
	  for (size_t j = 0; j < i; j++){
	sub[i] = sub[i] % prod[j];
      }
      sub[i] = (int)floor( (float)sub[i] / prod[i] );
    }
    delete [] prod;
  }

  static int sub2ind(std::vector<int> siz, std::vector<int> sub){
    size_t N = siz.size();
    int idx = 0;
	for (size_t i = 0; i < N; i++)
    {
      int prod = 1;
	  for (size_t j = N - 1; j > i; j--){
	prod *= siz[j];
      }
      idx += sub[i] * prod;
    }
    return idx;    
  }
  
  template <typename T>
  static double computeL2norm(std::vector<T> v1,std::vector<T> v2){
    size_t dim = v1.size();  
    if(dim != v2.size()){
      throw std::invalid_argument("The vector dimensions do not agree!");
    }
    double sum = 0;
    for(size_t i=0;i<dim;i++){
      double temp = v1[i]-v2[i];
      sum = sum+ temp*temp;
    }
    return sqrt(sum);
  }
  
  static bool isEqual(double a,double b, double threshold=EPSILON){
    return fabs(a-b)<=threshold;
  }
  
  static bool valueToVector(Json::Value& value,std::vector<double>& vector){
    if(value.isArray()){
      vector.resize(value.size());
      for(size_t i=0;i<value.size();i++){
	vector[i] = value[i].asDouble();
      }
      return true;
    }
    return false;
  }
};

#endif