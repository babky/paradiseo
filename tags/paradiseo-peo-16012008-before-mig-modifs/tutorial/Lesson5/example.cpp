/*
* <example.cpp>
* Copyright (C) DOLPHIN Project-Team, INRIA Futurs, 2006-2007
* (C) OPAC Team, LIFL, 2002-2007
*
* Sebastien Cahon, Alexandru-Adrian Tantar, Clive Canape
*
* This software is governed by the CeCILL license under French law and
* abiding by the rules of distribution of free software.  You can  use,
* modify and/ or redistribute the software under the terms of the CeCILL
* license as circulated by CEA, CNRS and INRIA at the following URL
* "http://www.cecill.info".
*
* As a counterpart to the access to the source code and  rights to copy,
* modify and redistribute granted by the license, users are provided only
* with a limited warranty  and the software's author,  the holder of the
* economic rights,  and the successive licensors  have only  limited liability.
*
* In this respect, the user's attention is drawn to the risks associated
* with loading,  using,  modifying and/or developing or reproducing the
* software by the user in light of its specific status of free software,
* that may mean  that it is complicated to manipulate,  and  that  also
* therefore means  that it is reserved for developers  and  experienced
* professionals having in-depth computer knowledge. Users are therefore
* encouraged to load and test the software's suitability as regards their
* requirements in conditions enabling the security of their systems and/or
* data to be ensured and,  more generally, to use and operate it in the
* same conditions as regards security.
* The fact that you are presently reading this means that you have had
* knowledge of the CeCILL license and that you accept its terms.
*
* ParadisEO WebSite : http://paradiseo.gforge.inria.fr
* Contact: paradiseo-help@lists.gforge.inria.fr
*
*/
// (c) OPAC Team, LIFL, 2007
//
// Contact: paradiseo-help@lists.gforge.inria.fr

// Libraries of problem
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


int main (int __argc, char * * __argv)
{

  // Example of a problem (TSP) with an hybridization

  peo :: init (__argc, __argv);


  loadParameters (__argc, __argv); /* Processing some parameters relative to the tackled
  				      problem (TSP) */

  RouteInit route_init; /* Its builds random routes */
  RouteEval full_eval; /* Full route evaluator */


  OrderXover order_cross; /* Recombination */
  PartialMappedXover pm_cross;
  EdgeXover edge_cross;
  CitySwap city_swap_mut;  /* Mutation */


  /** Local Search */
  TwoOptInit pmx_two_opt_init;
  TwoOptNext pmx_two_opt_next;
  TwoOptIncrEval pmx_two_opt_incr_eval;
  moBestImprSelect <TwoOpt> pmx_two_opt_move_select;
  moHC <TwoOpt> hc (pmx_two_opt_init, pmx_two_opt_next, pmx_two_opt_incr_eval, pmx_two_opt_move_select, full_eval);

  /** The EA */
  eoPop <Route> ox_pop (POP_SIZE, route_init);  /* Population */

  eoGenContinue <Route> ox_cont (NUM_GEN); /* A fixed number of iterations */
  eoCheckPoint <Route> ox_checkpoint (ox_cont); /* Checkpoint */
  peoSeqPopEval <Route> ox_pop_eval (full_eval);
  eoStochTournamentSelect <Route> ox_select_one;
  eoSelectNumber <Route> ox_select (ox_select_one, POP_SIZE);
  eoSGATransform <Route> ox_transform (order_cross, CROSS_RATE, city_swap_mut, MUT_RATE);
  peoSeqTransform <Route> ox_para_transform (ox_transform);
  eoEPReplacement <Route> ox_replace (2);

  peoEA <Route> ox_ea (ox_checkpoint, ox_pop_eval, ox_select, ox_para_transform, ox_replace);

  ox_ea (ox_pop);   /* Application to the given population */


  peo :: run ();
  peo :: finalize (); /* Termination */

  if (getNodeRank()==1)
    std :: cout << "\nResult : "<<ox_pop[ 0 ].fitness();
  hc( ox_pop[ 0 ] );
  if (getNodeRank()==1)
    std :: cout << "\n\nAfter an hybridization : " << ox_pop[ 0 ].fitness() << std :: endl;


  return 0;
}
