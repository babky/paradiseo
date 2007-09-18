// (c) OPAC Team, LIFL, July 2007
//
// Contact: paradiseo-help@lists.gforge.inria.fr

#include "param.h"
#include "route_init.h"
#include "route_eval.h"

#include "order_xover.h"
#include "edge_xover.h"
#include "partial_mapped_xover.h"
#include "city_swap.h"
#include "part_route_eval.h"
#include "merge_route_eval.h"
#include "two_opt_init.h"
#include "two_opt_next.h"
#include "two_opt_incr_eval.h"

#include <peo>

#define POP_SIZE 10
#define NUM_GEN 10
#define CROSS_RATE 1.0
#define MUT_RATE 0.01

#define NUM_PART_EVALS 2


int main (int __argc, char * * __argv) {

  peo :: init (__argc, __argv);

  
  loadParameters (__argc, __argv); /* Processing some parameters relative to the tackled
				      problem (TSP) */

  RouteInit route_init; /* Its builds random routes */  
  RouteEval full_eval; /* Full route evaluator */


  MergeRouteEval merge_eval;

  std :: vector <eoEvalFunc <Route> *> part_eval;
  for (unsigned i = 1 ; i <= NUM_PART_EVALS ; i ++)
    part_eval.push_back (new PartRouteEval ((float) (i - 1) / NUM_PART_EVALS, (float) i / NUM_PART_EVALS));

  
  OrderXover order_cross; /* Recombination */
  CitySwap city_swap_mut;  /* Mutation */


  /** The EA */
  eoPop <Route> ox_pop (POP_SIZE, route_init);  /* Population */
  
  eoGenContinue <Route> ox_cont (NUM_GEN); /* A fixed number of iterations */  
  eoCheckPoint <Route> ox_checkpoint (ox_cont); /* Checkpoint */
  peoParaPopEval <Route> ox_pop_eval (full_eval);  
  eoStochTournamentSelect <Route> ox_select_one;
  eoSelectNumber <Route> ox_select (ox_select_one, POP_SIZE);
  eoSGATransform <Route> ox_transform (order_cross, CROSS_RATE, city_swap_mut, MUT_RATE);
  peoSeqTransform <Route> ox_para_transform (ox_transform);    
  eoEPReplacement <Route> ox_replace (2);

  
  peoEA <Route> ox_ea (ox_checkpoint, ox_pop_eval, ox_select, ox_para_transform, ox_replace);

    
  ox_ea (ox_pop);   /* Application to the given population */
    
  peo :: run ();
  peo :: finalize (); /* Termination */
  
 

  // rank 0 is assigned to the scheduler in the XML mapping file
  if ( getNodeRank() == 1 ) {

    std::cout << "EA[ 0 ] -----> " << ox_pop.best_element().fitness() << std::endl;
  }
 
    
  return 0;
}
