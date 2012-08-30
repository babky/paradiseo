// "peoParaSGATransform.h"

//(c) OPAC Team, LIFL, August 2005

/* 
   Contact: paradiseo-help@lists.gforge.inria.fr
*/

#ifndef __peoParaSGATransform_h
#define __peoParaSGATransform_h

#include "peoTransform.h"
#include "core/thread.h"
#include "core/messaging.h"
#include "core/peo_debug.h"


extern int getNodeRank();


template< class EOT > class peoParaSGATransform : public peoTransform< EOT > {

public:

	using peoTransform< EOT > :: requestResourceRequest;
	using peoTransform< EOT > :: resume;
	using peoTransform< EOT > :: stop;
	using peoTransform< EOT > :: getOwner;

	peoParaSGATransform( 

				eoQuadOp< EOT >& __cross,
				double __cross_rate,
				eoMonOp< EOT >& __mut, 
				double __mut_rate 
	);

	void operator()( eoPop< EOT >& __pop );
	
	void packData();
	
	void unpackData();
	
	void execute();
	
	void packResult();
	
	void unpackResult();
	
	void notifySendingData();
	void notifySendingAllResourceRequests();

private:

    eoQuadOp< EOT >& cross;
    double cross_rate;

    eoMonOp< EOT >& mut;
    double mut_rate;

    unsigned idx;

    eoPop< EOT >* pop;

    EOT father, mother;

    unsigned num_term;
};

template< class EOT > peoParaSGATransform< EOT > :: peoParaSGATransform( 

				eoQuadOp< EOT >& __cross,
				double __cross_rate,
				eoMonOp < EOT >& __mut,
				double __mut_rate 

		) : cross( __cross ), cross_rate( __cross_rate ), mut( __mut ), mut_rate( __mut_rate )
{

}


template< class EOT > void peoParaSGATransform< EOT > :: packData() {

	pack( idx );
	 :: pack( pop->operator[]( idx++ ) );
	 :: pack( pop->operator[]( idx++ ) );
}


template< class EOT > void peoParaSGATransform< EOT > :: unpackData() {

	unpack( idx );
	 :: unpack( father );
	 :: unpack( mother );
}


template< class EOT > void peoParaSGATransform< EOT > :: execute() {

	if( rng.uniform() < cross_rate ) cross( mother, father );

	if( rng.uniform() < mut_rate ) mut( mother );
	if( rng.uniform() < mut_rate ) mut( father );
}


template< class EOT > void peoParaSGATransform< EOT > :: packResult() {

	pack( idx );
	 :: pack( father );
	 :: pack( mother );
}


template< class EOT > void peoParaSGATransform< EOT > :: unpackResult() {

	unsigned sidx;
	
	unpack( sidx );
	 :: unpack( pop->operator[]( sidx++ ) );
	 :: unpack( pop->operator[]( sidx ) );
	num_term += 2;
	
	if( num_term == pop->size() ) {

		getOwner()->setActive();
		resume();
	}
}


template< class EOT > void peoParaSGATransform< EOT > :: operator()( eoPop < EOT >& __pop ) {

	printDebugMessage( "performing the parallel transformation step." );
	pop = &__pop;
	idx = 0;
	num_term = 0;
	requestResourceRequest( __pop.size() / 2 );
	stop();
}


template< class EOT > void peoParaSGATransform< EOT > :: notifySendingData() {

}


template< class EOT > void peoParaSGATransform< EOT > :: notifySendingAllResourceRequests() {

	getOwner()->setPassive();
}


#endif
