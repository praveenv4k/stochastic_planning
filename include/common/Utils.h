/*! 
 *  \author    PraveenKumar Vasudevan
 *  \date      2014
 */
#ifndef __UTILS_H__
#define __UTILS_H__

#include <stdexcept>
#include <math.h>
#include "json/json.h"
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/unordered_map.hpp>
#include <Container.h>
#include <map>

#define EPSILON 1e-6

//ind2sub and sub2ind functions in the Utils class are taken from 
//https://bitbucket.org/kritisen/utilitiescpp
/**
 * @brief Utility class
 **/
class Utils{
public:
  /**
   * @brief Index to sub indices (matrix index) - Analogous to Matlab function of the same name
   *
   * @param siz Size of the space along each dimensions
   * @param idx Index
   * @param sub Matrix indices
   * @return void
   **/
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

  /**
   * @brief Matrix indices to single unique index - Analogous to Matlab function of the same name
   *
   * @param siz Size along each dimensions
   * @param sub Matrix indices
   * @return int - Unique index
   **/
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
  /**
   * @brief Computes the L2 norm of two vectors
   *
   * @param v1 Vector container
   * @param v2 Vector container
   * @return double - L2 norm
   **/
  static double computeL2norm(T v1,T v2){
    size_t dim = v1.size();  
    if(dim != v2.size()){
      throw std::invalid_argument("The vector dimensions do not agree!");
    }
    double sum = 0.0;
    for(size_t i=0;i<dim;i++){
      double temp = v1[i]-v2[i];
      sum = sum+ temp*temp;
    }
    return sqrt(sum);
  }
  
  /**
   * @brief Check equality of two floating pt numbers
   *
   * @param a Value 1
   * @param b Value 2
   * @param threshold Threshold value Defaults to 1e-6.
   * @return bool - True if equal,false otherwise
   **/
  static bool isEqual(double a,double b, double threshold=EPSILON){
    return fabs(a-b)<=threshold;
  }
  
  template <typename T>
  /**
   * @brief Convert a JSON value to vector
   *
   * @param value JSON Value
   * @param vector Output Vector
   * @return bool True on successful conversion,false otherwise
   **/
  static bool valueToVector(Json::Value& value,T& vector){
    if(!value.isNull() && value.isArray()){
      vector.resize(value.size());
      for(Json::ArrayIndex i=0;i<value.size();i++){
	vector[i] = value[i].asDouble();
      }
      return true;
    }
    return false;
  }
  
  template <typename T>
  /**
   * @brief Convert degree to radian
   *
   * @param deg Value in degree
   * @return T Value in radian
   **/
  static T deg2Radian(T deg){
    return deg*M_PI/180.0;
  }
  
  /**
   * @brief concatenate one vector to another
   *
   * @param a Vector 1
   * @param b Vector 2
   * @return :vector< double, std::allocator< double > > - Concatenated vector
   **/
  static std::vector<double> concatenate(std::vector<double>& a,std::vector<double>& b){
    std::vector<double> ret;
    for(std::vector<double>::iterator it= a.begin();it!=a.end();it++){
      ret.push_back(*it);
    }
    for(std::vector<double>::iterator it= b.begin();it!=b.end();it++){
      ret.push_back(*it);
    }
    return ret;
  }
  
  template<typename T>
  /**
   * @brief Save the unordered map
   *
   * @param name File name
   * @param mp map
   * @return void
   **/
  static void SaveMap(const std::string& name, T mp)
  {
      std::ofstream fl(name.c_str(),std::ios_base::binary);
      boost::archive::text_oarchive oa(fl);
      boost::serialization::save(oa,mp,0);
  }

  template<typename T>
  /**
   * @brief Load an unordered map from a file
   *
   * @param name Name of the file
   * @param mp Unoordered map
   * @return void
   **/
  static void LoadMap(std::string& name, T mp)
  {
      std::ifstream fl(name.c_str(),std::ios_base::binary);
      boost::archive::text_iarchive oa(fl);
      boost::serialization::load(oa,mp,0);
  }
  
  template<typename T>
  /**
   * @brief Dot product of two vectors
   *
   * @param a Vector 1
   * @param b Vector 2
   * @return T Dot product value
   **/
  static T dot(Container<T> a,Container<T> b){
    if(a.size() != b.size()){
      throw std::invalid_argument("The vector dimensions do not agree!");
    }
    T dotP=0;
    for(size_t i=0;i<a.size();i++){
      dotP+= a[i]*b[i];
    }
    return dotP;
  }
  
  template<typename T>
  /**
   * @brief Shortest Distance of point to a 3d line
   *
   * @param pt 3d point
   * @param seg 3d line segment
   * @return T Distance
   **/
  static T ptDistToLine(Container<T> pt, 
				      std::pair<Container<T>,Container<T> > seg)
  {
      Container<T> v = seg.second - seg.first;
      Container<T> w = pt - seg.first;

      T c1 = Utils::dot(w,v);
      if ( c1 <= 0 )
	    return computeL2norm(pt, seg.first);

      T c2 = Utils::dot(v,v);
      if ( c2 <= c1 )
	    return computeL2norm(pt, seg.second);

      T b = c1 / c2;
      Container<T> pb = seg.first + (v*b);
      return computeL2norm(pt, pb);
  }
  
  template <typename T>
  /**
   * @brief Given a point,angle and length, it computes a new point at angle separated by a distance specified by length
   *
   * @param pt2d 2D point
   * @param angle Angle
   * @param length Length
   * @return Container< T > - New 2D point
   **/
  static Container<T> compute2dPoint(Container<T> pt2d,T angle,T length){
    if(pt2d.size() > 2){
      throw std::invalid_argument("The vector should be 2 dimensional!");
    }
    Container<T> ret;
    ret.resize(2);
    std::cout << "Pt: " << pt2d << "; Angle " << angle << "; (C,S) " << cos(deg2Radian(angle)) << " " << sin(deg2Radian(angle)) << std::endl;
    ret[0] = pt2d[0] + cos(deg2Radian(angle))*length;
    ret[1] = pt2d[1] + sin(deg2Radian(angle))*length;
    return ret;
  }
};

#endif