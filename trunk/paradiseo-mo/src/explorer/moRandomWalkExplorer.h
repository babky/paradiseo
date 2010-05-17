/*
  <moRandomWalkExplorer.h>
  Copyright (C) DOLPHIN Project-Team, INRIA Lille - Nord Europe, 2006-2010

  Sébastien Verel, Arnaud Liefooghe, Jérémie Humeau

  This software is governed by the CeCILL license under French law and
  abiding by the rules of distribution of free software.  You can  use,
  modify and/ or redistribute the software under the terms of the CeCILL
  license as circulated by CEA, CNRS and INRIA at the following URL
  "http://www.cecill.info".

  As a counterpart to the access to the source code and  rights to copy,
  modify and redistribute granted by the license, users are provided only
  with a limited warranty  and the software's author,  the holder of the
  economic rights,  and the successive licensors  have only  limited liability.

  In this respect, the user's attention is drawn to the risks associated
  with loading,  using,  modifying and/or developing or reproducing the
  software by the user in light of its specific status of free software,
  that may mean  that it is complicated to manipulate,  and  that  also
  therefore means  that it is reserved for developers  and  experienced
  professionals having in-depth computer knowledge. Users are therefore
  encouraged to load and test the software's suitability as regards their
  requirements in conditions enabling the security of their systems and/or
  data to be ensured and,  more generally, to use and operate it in the
  same conditions as regards security.
  The fact that you are presently reading this means that you have had
  knowledge of the CeCILL license and that you accept its terms.

  ParadisEO WebSite : http://paradiseo.gforge.inria.fr
  Contact: paradiseo-help@lists.gforge.inria.fr
*/

#ifndef _moRandomWalkexplorer_h
#define _moRandomWalkexplorer_h

#include <explorer/moNeighborhoodExplorer.h>
#include <comparator/moNeighborComparator.h>
#include <comparator/moSolNeighborComparator.h>
#include <neighborhood/moNeighborhood.h>

/**
 * Explorer for a random walk explorer
 */
template< class Neighbor >
class moRandomWalkExplorer : public moNeighborhoodExplorer<Neighbor>
{
public:
    typedef typename Neighbor::EOT EOT ;
    typedef moNeighborhood<Neighbor> Neighborhood ;

    using moNeighborhoodExplorer<Neighbor>::neighborhood;
    using moNeighborhoodExplorer<Neighbor>::eval;

    /**
     * Constructor
     * @param _neighborhood the neighborhood
     * @param _eval the evaluation function
     * @param _nbStep maximum number of step to do
     */
    moRandomWalkExplorer(Neighborhood& _neighborhood, moEval<Neighbor>& _eval, unsigned _nbStep) : moNeighborhoodExplorer<Neighbor>(_neighborhood, _eval), nbStep(_nbStep) {
        isAccept = false;
        current=new Neighbor();
        // number of step done
        step = 0;
        if (!neighborhood.isRandom()) {
            std::cout << "moRandomWalkExplorer::Warning -> the neighborhood used is not random" << std::endl;
        }
    }

    /**
     * Destructor
     */
    ~moRandomWalkExplorer() {
        delete current;
    }

    /**
     * initialization of the number of step to be done
     * @param _solution a solution (unused)
     */
    virtual void initParam(EOT & _solution) {
        step     = 0;
        isAccept = true;
    };

    /**
     * increase the number of step
     * @param _solution a solution (unused)
     */
    virtual void updateParam(EOT & _solution) {
        step++;
    };

    /**
     * terminate: NOTHING TO DO
     * @param _solution a solution (unused)
     */
    virtual void terminate(EOT & _solution) {};

    /**
     * Explore the neighborhood with only one random solution
     * we supposed that the first neighbor is uniformly selected in the neighborhood
     * @param _solution a solution
     */
    virtual void operator()(EOT & _solution) {

        //Test if _solution has a Neighbor
        if (neighborhood.hasNeighbor(_solution)) {
            //init the first neighbor
            neighborhood.init(_solution, (*current));

            //eval the _solution moved with the neighbor and stock the result in the neighbor
            eval(_solution, (*current));

            isAccept = true;
        }
        else {
            //if _solution hasn't neighbor,
            isAccept=false;
        }
    };

    /**
     * continue if there is a neighbor and it is remainds some steps to do
     * @param _solution the solution
     * @return true there is some steps to do
     */
    virtual bool isContinue(EOT & _solution) {
        return (step < nbStep)  && isAccept ;
    };

    /**
     * move the solution with the best neighbor
     * @param _solution the solution to move
     */
    virtual void move(EOT & _solution) {
        //move the solution
        (*current).move(_solution);
        //update its fitness
        _solution.fitness((*current).fitness());
    };

    /**
     * accept test if an amelirated neighbor was be found
     * @param _solution the solution
     * @return true if the best neighbor ameliorate the fitness
     */
    virtual bool accept(EOT & _solution) {
        if (neighborhood.hasNeighbor(_solution))
            isAccept = true ;
        return isAccept;
    };

private:
    // current number of step
    unsigned int step;

    // maximum number of steps to do
    unsigned int nbStep;

    //Pointer on the best and the current neighbor
    Neighbor* current;

    // true if the move is accepted
    bool isAccept ;
};


#endif
