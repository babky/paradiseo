// -*- mode: c++; c-indent-level: 4; c++-member-init-indent: 8; comment-column: 35; -*-

// "graph.cpp"

// (c) OPAC Team, LIFL, 2003-2006

/* LICENCE TEXT 
   
   Contact: paradiseo-help@lists.gforge.inria.fr
*/

#include <fstream>
#include <iostream>
#include <math.h>

#include "graph.h"

namespace Graph {

  static std :: vector <std :: pair <double, double> > vectCoord ; // Coordinates
  
  static std :: vector <std :: vector <unsigned int> > dist ; // Distances Mat.

  unsigned size () 
  {
    return dist.size () ;
  }

  void computeDistances () 
  {
    
    // Dim.
    unsigned int numCities = vectCoord.size () ;
    dist.resize (numCities) ;
    for (unsigned int i = 0 ; i < dist.size () ; i ++)
      {
	dist [i].resize (numCities) ;
      }
    
    // Computations.
    for (unsigned int i = 0 ; i < dist.size () ; i ++)
      {
	for (unsigned int j = i + 1 ; j < dist.size () ; j ++) 
	  {
	    double distX = (double)(vectCoord [i].first - vectCoord [j].first) ;
	    double distY = (double)(vectCoord [i].second - vectCoord [j].second) ;
	    dist [i] [j] = dist [j] [i] = (unsigned) (sqrt ((float) (distX * distX + distY * distY)) + 0.5) ;
	  }
      }
  }

  void load (const char * __fileName) 
  {
    
    std :: ifstream f (__fileName) ;
    
    std :: cout << ">> Loading [" << __fileName << "]" << std :: endl ;
    
    if (f) 
      {
	unsigned int num_vert ; 
	
	f >> num_vert ;
	vectCoord.resize (num_vert) ;
	
	for (unsigned int i = 0 ; i < num_vert ; i ++)	
	  {
	    f >> vectCoord [i].first >> vectCoord [i].second ;
	  }
                  
	f.close () ;
	
	computeDistances () ;
      }
    else 
      {
	
	std :: cout << __fileName << " doesn't exist !!!" << std :: endl ;
	// Bye !!!
	exit (1) ;
      }
  }
  
  float distance (unsigned int __from, unsigned int __to) 
  {
    return (float)(dist [__from] [__to]) ;
  }
}


