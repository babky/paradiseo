/*
* <moeoFixedSizeArchive.h>
* Copyright (C) DOLPHIN Project-Team, INRIA Lille-Nord Europe, 2006-2008
* (C) OPAC Team, LIFL, 2002-2008
*
* Arnaud Liefooghe
* Jeremie Humeau
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
//-----------------------------------------------------------------------------
// moeoFixedSizeArchive.h
//-----------------------------------------------------------------------------

#ifndef MOEOFIXEDSIZEARCHIVE_H_
#define MOEOFIXEDSIZEARCHIVE_H_

#include <eoPop.h>

/**
 * An FixedSizeArchive is a secondary population that stores non-dominated solutions whith a fixed size.
 */
template < class MOEOT >
class moeoFixedSizeArchive : public moeoArchive < MOEOT >
{
public:

    /**
     * The type of an objective vector for a solution
     */
    typedef typename MOEOT::ObjectiveVector ObjectiveVector;


    /**
     * Default ctor.
     * The moeoObjectiveVectorComparator used to compare solutions is based on Pareto dominance
     */
    moeoFixedSizeArchive() : moeoArchive < MOEOT >() {}


    /**
     * Ctor
     * @param _comparator the moeoObjectiveVectorComparator used to compare solutions
     */
    moeoFixedSizeArchive(moeoObjectiveVectorComparator < ObjectiveVector > & _comparator) : moeoArchive < MOEOT >( _comparator) {}


    /**
     * Returns true if the current archive dominates _objectiveVector according to the moeoObjectiveVectorComparator given in the constructor
     * @param _objectiveVector the objective vector to compare with the current archive
     */

    /**
     * Updates the archive with a given individual _moeo
     * @param _moeo the given individual
     */
    virtual void operator()(const MOEOT & _moeo)=0;

    /**
     * Updates the archive with a given population _pop
     * @param _pop the given population
     */
    virtual void operator()(const eoPop < MOEOT > & _pop)=0;

};

#endif /*MOEOFIXEDSIZEARCHIVE_H_*/
