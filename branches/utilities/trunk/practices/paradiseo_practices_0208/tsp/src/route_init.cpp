// -*- mode: c++; c-indent-level: 4; c++-member-init-indent: 8; comment-column: 35; -*-

// "route_init.cpp"

// (c) OPAC Team, LIFL, 2003-2006

/* LICENCE TEXT
   
   Contact: paradiseo-help@lists.gforge.inria.fr
*/

#include <utils/eoRNG.h>

#include "route_init.h"
#include "graph.h"

void RouteInit :: operator () (Route & __route) 
{
  
  // Init.
  __route.clear () ;
  for (unsigned int i = 0 ; i < Graph :: size () ; i ++)
    {
      __route.push_back (i) ;
    }
  
  // Swap. cities

  for (unsigned int i = 0 ; i < Graph :: size () ; i ++) 
    {
      //unsigned int j = rng.random (Graph :: size ()) ;
      
      unsigned int j = (unsigned int) (Graph :: size () * (rand () / (RAND_MAX + 1.0))) ;
      unsigned int city = __route [i] ;
      __route [i] = __route [j] ;
      __route [j] = city ;
    }   
}
