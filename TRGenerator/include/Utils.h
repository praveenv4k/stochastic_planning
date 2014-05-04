#ifndef __UTILS_H__
#define __UTILS_H__

//Part of the functions in the Utils class are taken from 
//https://bitbucket.org/kritisen/utilitiescpp
class Utils{
public:
  static void ind2sub(std::vector<int> siz, int idx, std::vector<int>& sub){
    size_t N = siz.size();
    int *prod = new int [N];
    for (int i = 0; i < N; i++){
      prod[i] = 1;
      for (int j = N-1; j > i; j--){
	prod[i] *= siz[j];	
      }
    }
    sub.resize(N);
    for (int i = 0; i < N; i++){
      sub[i] = idx ;
      for (int j = 0; j < i ; j++)
	sub[i] = sub[i] % prod[j];
      sub[i] = (int)floor( (float)sub[i] / prod[i] );
    }
    delete [] prod;
  }

  static int sub2ind(std::vector<int> siz, std::vector<int> sub){
    size_t N = siz.size();
    int idx = 0;
    for (int i = 0; i < N; i++)
    {
	    int prod = 1;
	    for (int j = N-1; j > i; j--)
		    prod *= siz[j];
	    idx += sub[i] * prod;
    }
    return idx;    
  }
  
};

#endif