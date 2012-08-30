// "main.cpp"

// (c) OPAC Team, LIFL, January 2006

/* 
   Contact: paradiseo-help@lists.gforge.inria.fr
*/

#include "route.h"
#include "route_init.h"
#include "route_eval.h"

#include "order_xover.h"
#include "city_swap.h"

#include "param.h"

#include "merge_route_eval.h"
#include "part_route_eval.h"


#include <paradiseo>


#define POP_SIZE 10
#define NUM_GEN 100
#define CROSS_RATE 1.0
#define MUT_RATE 0.01

#define NUM_PART_EVALS 2


// by default, parallel evaluation of the population is performed;
// for parallel fitness evaluation, uncomment the following line

// #define PARALLEL_FIT_EVALUATION


int main( int __argc, char** __argv ) {

	// initializing the ParadisEO-PEO environment
	peo :: init( __argc, __argv );


	// processing the command line specified parameters
	loadParameters( __argc, __argv );


	// init, eval operators, EA operators -------------------------------------------------------------------------------------------------------------

	RouteInit route_init;	// random init object - creates random Route objects
	RouteEval full_eval;	// evaluator object - offers a fitness value for a specified Route object

	OrderXover crossover;	// crossover operator - creates two offsprings out of two specified parents
	CitySwap mutation;	// mutation operator - randomly mutates one gene for a specified individual
	// ------------------------------------------------------------------------------------------------------------------------------------------------


	// evolutionary algorithm components --------------------------------------------------------------------------------------------------------------

	eoPop< Route > population( POP_SIZE, route_init );	// initial population for the algorithm having POP_SIZE individuals


	#ifdef PARALLEL_FIT_EVALUATION

		MergeRouteEval merge_eval;

		std :: vector< eoEvalFunc< Route >* > part_eval;
		for ( unsigned index = 1; index <= NUM_PART_EVALS; index++ )
			part_eval.push_back( new PartRouteEval( ( float )( index - 1 ) / NUM_PART_EVALS, ( float )index / NUM_PART_EVALS ) );

		peoParaPopEval< Route > ox_pop_eval( part_eval, merge_eval );

	#else

               peoParaPopEval< Route > ox_pop_eval( full_eval );

	#endif



	peoParaPopEval< Route > eaPopEval( full_eval );		// evaluator object - to be applied at each iteration on the entire population

	eoGenContinue< Route > eaCont( NUM_GEN );		// continuation criterion - the algorithm will iterate for NUM_GEN generations
	eoCheckPoint< Route > eaCheckpointContinue( eaCont );	// checkpoint object - verify at each iteration if the continuation criterion is met

	eoRankingSelect< Route > selectionStrategy;		// selection strategy - applied at each iteration for selecting parent individuals
	eoSelectNumber< Route > eaSelect( selectionStrategy, POP_SIZE ); // selection object - POP_SIZE individuals are selected at each iteration

	// transform operator - includes the crossover and the mutation operators with a specified associated rate
	eoSGATransform< Route > transform( crossover, CROSS_RATE, mutation, MUT_RATE );
	peoSeqTransform< Route > eaTransform( transform );	// ParadisEO transform operator (please remark the peo prefix) - wraps an e EO transform object

	eoPlusReplacement< Route > eaReplace;			// replacement strategy - for replacing the initial population with offspring individuals
	// ------------------------------------------------------------------------------------------------------------------------------------------------


	// ParadisEO-PEO evolutionary algorithm -----------------------------------------------------------------------------------------------------------

	peoEA< Route > eaAlg( eaCheckpointContinue, eaPopEval, eaSelect, eaTransform, eaReplace );
	
	eaAlg( population );	// specifying the initial population for the algorithm, to be iteratively evolved
	// ------------------------------------------------------------------------------------------------------------------------------------------------


	peo :: run( );
	peo :: finalize( );
	// shutting down the ParadisEO-PEO environment

	return 0;
}
