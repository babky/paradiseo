/*
* <moeoUnifiedDominanceBasedLS.h>
* Copyright (C) DOLPHIN Project-Team, INRIA Futurs, 2006-2008
* (C) OPAC Team, LIFL, 2002-2008
*
* Arnaud Liefooghe
* Jérémie Humeau
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

#ifndef _MOEOUNIFIEDDOMINANCEBASEDLS_H
#define _MOEOUNIFIEDDOMINANCEBASEDLS_H

#include <eo>
#include <moeo>
#include <moeoPopLS.h>
#include <moeoPopNeighborhoodExplorer.h>
#include <moeoUnvisitedSelect.h>

template < class Move >
class moeoUnifiedDominanceBasedLS : public moeoPopLS < Move >
{
public:

    typedef typename Move::EOType MOEOT;


    moeoUnifiedDominanceBasedLS(
        eoContinue < MOEOT > & _continuator,
        eoEvalFunc < MOEOT > & _eval,
        moeoArchive < MOEOT > & _archive,
        moeoPopNeighborhoodExplorer < Move > & _explorer,
        moeoUnvisitedSelect < MOEOT > & _select) :
            continuator(_continuator), loopEval(_eval), popEval(loopEval), archive(_archive), explorer(_explorer), select(_select) {}


    /**
     * Applies a few generation of evolution to the population _pop.
     * @param _pop the population
     */
    virtual void operator()(eoPop < MOEOT > & _pop)
    {
    	std::vector < unsigned int> selectionVector;
        eoPop < MOEOT > tmp_pop;
        popEval(tmp_pop, _pop);
        archive(_pop);
        do{
            tmp_pop.resize(0);
            //selection des individus non visités à explorer
            selectionVector = select(archive);
            //exploration
            //explorer(archive, selectionVector, tmp_pop);
            explorer(archive, selectionVector, tmp_pop);
            //mise à jour de la pop ou archive
            archive(tmp_pop);
        }
        while (continuator(archive) && naturalContinuator(archive));
//         std::cout << "Final archive\n";
//         archive.sortedPrintOn(std::cout);
//         std::cout << std::endl;
    }


protected:

    eoContinue < MOEOT > & continuator;
    eoPopLoopEval < MOEOT > loopEval;
    eoPopEvalFunc < MOEOT > & popEval;
    moeoArchive < MOEOT > & archive;
    moeoPopNeighborhoodExplorer < Move > & explorer;
    moeoUnvisitedSelect < MOEOT > & select;

class moeoContinue : public eoUF < eoPop < MOEOT > &, bool >
    {
    public:

        moeoContinue(){}

        virtual bool operator()(eoPop < MOEOT > & _pop)
        {
            bool res = false;
            unsigned int i=0;
            while (!res && i < _pop.size()){
                res = (_pop[i].flag() != 1);
                i++;
            }
            return res;
        }
    } naturalContinuator;

};

#endif /*MOEOUNIFIEDDOMINANCEBASEDLS_H_*/

