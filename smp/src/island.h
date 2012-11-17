/*
<island.h>
Copyright (C) DOLPHIN Project-Team, INRIA Lille - Nord Europe, 2006-2012

Alexandre Quemy, Thibault Lasnier - INSA Rouen

This software is governed by the CeCILL license under French law and
abiding by the rules of distribution of free software.  You can  ue,
modify and/ or redistribute the software under the terms of the CeCILL
license as circulated by CEA, CNRS and INRIA at the following URL
"http://www.cecill.info".

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

#ifndef ISLAND_H_
#define ISLAND_H_

#include <queue>
#include <vector>
#include <utility>

#include <eo>
#include <abstractIsland.h>
#include <migPolicy.h>
#include <intPolicy.h>
#include <PPExpander.h>
#include <contWrapper.h>
#include <contDispatching.h>

namespace paradiseo
{
namespace smp
{

template<template <class> class EOAlgo, class EOT>
class Island : private ContWrapper<EOT>, public AIsland<EOT>
{
public:
    /**
     * Constructor
     * @param _popSize Size of the algorithm population.
     * @param _chromInit Population initializer.
     * @param _intPolicy Integration policy
     * @param _migPolicy Migration policy
     * @param args Parameters to construct the algorithm of the island.
     */
    template<class... Args>
    Island(unsigned _popSize, eoInit<EOT>& _chromInit, IntPolicy<EOT>& _intPolicy, MigPolicy<EOT>& _migPolicy, Args&... args);
    
    /**
     * Start the island.
     */
    void operator()();
    
    /**
     * Update the list of imigrants.
     * @param _data Elements to integrate in the main population.
     */
    void update(eoPop<EOT>& _data);
    
    /**
     * Return a reference to the island population.
     * @return Reference to the island population
     */
    eoPop<EOT>& getPop();
    
    /**
     * Check if there is population to receive or to migrate
     */
    virtual void check(void);
    
protected:

    /**
     * Send population to mediator
     * @param _select Method to select EOT to send
     */
    virtual void send(eoSelect<EOT>& _select);
    
    /**
     * Check if there is population to receive
     */
    virtual void receive(void);
    
    eoPop<EOT> pop;
    EOAlgo<EOT> algo;
    std::queue<eoPop<EOT>*> listImigrants;
    IntPolicy<EOT>& intPolicy;
    MigPolicy<EOT>& migPolicy;
};

#include <island.cpp>

}

}

#endif