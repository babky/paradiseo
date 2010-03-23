/*
* <t-moILS.cpp>
* Copyright (C) DOLPHIN Project-Team, INRIA Futurs, 2006-2007
* (C) OPAC Team, LIFL, 2002-2008
*
* Sébastien Cahon, Jean-Charles Boisson (Jean-Charles.Boisson@lifl.fr)
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
// t-moILS.cpp
//-----------------------------------------------------------------------------

#include <eo>  // EO
#include <mo>  // MO

using std::cout;
using std::endl;

//-----------------------------------------------------------------------------

typedef EO<unsigned int> solution;

class testMove : public moMove <solution>
{
public :
  void operator () (solution & _solution)
  {
    solution sol=_solution;
  }
} ;

class solutionAlgo : public moAlgo <solution>
{
public :
  bool operator () (solution & _solution)
  {
    solution solution(_solution);
    return true;
  }
} ;

class solutionContinue : public moSolContinue<solution>
{
public :
  solutionContinue(): counter(0)
  {}

  bool operator () (const solution & _solution)
  {
    const solution sol(_solution);
    
    if(counter<2)
      {
	counter++;
	return true;
      }
    return false;
  }

  void init()
  {}
private :
  unsigned int counter;
} ;

class solutionComparator : public moComparator<solution>
{
public :
  
  solutionComparator() : counter(0)
  {}

  bool operator () (const solution & _solution1 , const solution & _solution2)
  {
    const solution sol1(_solution1);
    const solution sol2(_solution2);

    if(counter<2)
      {
	counter++;
	return false;
      }

    return true;
  }
private :
  unsigned int counter;
} ;

class solutionPerturbation : public eoMonOp<solution>
{
public :
  bool operator () (solution & _solution)
  {
    _solution.fitness(2);

    return true;
  }
} ;

class solutionEval : public eoEvalFunc <solution>
{
public :
  void operator () (solution & _solution)
  {
    solution solution(_solution);
    _solution.fitness(2);
  }
} ;

//-----------------------------------------------------------------------------

int
main()
{
  std::string test_result;
  int return_value;

  solution solution;
  
  solutionAlgo algorithm;
  solutionContinue continu;
  solutionComparator comparator;
  solutionPerturbation perturbation;
  solutionEval eval;
  
  moILS<testMove> ils(algorithm, continu, comparator, perturbation, eval);

  cout << "[ moILS                        ] ==> ";
  
  solution.fitness(0);

  ils(solution);

  test_result=((solution.fitness()!=2)?"KO":"OK");
  return_value=((test_result.compare("KO")==0)?EXIT_FAILURE:EXIT_SUCCESS);

  cout << test_result << endl;
  return return_value;
}

//-----------------------------------------------------------------------------
