// "edge_xover.h"

// (c) OPAC Team, LIFL, 2003

/* 
   Contact: paradiseo-help@lists.gforge.inria.fr
*/

#ifndef edge_xover_h
#define edge_xover_h

#include <vector>
#include <set>

#include <eoOp.h>

#include "route.h"

/** Edge Crossover */
class EdgeXover : public eoQuadOp <Route> {
  
public :
  
  bool operator () (Route & __route1, Route & __route2) ;

private :
  
  void cross (const Route & __par1, const Route & __par2, Route & __child) ; /* Binary */

  void remove_entry (unsigned __vertex, std :: vector <std :: set <unsigned> > & __map) ;
  /* Updating the map of entries */

  void build_map (const Route & __par1, const Route & __par2) ;

  void add_vertex (unsigned __vertex, Route & __child) ;

  std :: vector <std :: set <unsigned> > _map ; /* The handled map */

  std :: vector <bool> visited ; /* Vertices that are already visited */

} ;

#endif
