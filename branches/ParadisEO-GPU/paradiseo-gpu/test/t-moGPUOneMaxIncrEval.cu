/*
  <t-OneMaxIncrEval.cu>
  Copyright (C) DOLPHIN Project-Team, INRIA Lille - Nord Europe, 2006-2012

  Karima Boufaras, Thé Van LUONG

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

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <neighborhood/moGPUBitNeighbor.h>
#include <GPUType/moGPUBitVector.h>
#include <problems/eval/moGPUEvalOneMax.h>
#include <problems/eval/moGPUOneMaxIncrEval.h>

#define NB_POS 1

typedef moGPUBitVector<eoMaximizingFitness> Solution;
typedef moGPUBitNeighbor <Solution,eoMaximizingFitness> Neighbor;

int main() {


  std::cout << "[t-moGPUOneMaxIncrEval] => START" << std::endl;


  Solution sol(5);
  moGPUEvalOneMax<Solution> eval;
  moGPUOneMaxIncrEval<Neighbor> incr_eval;
  int sum=0;
  int fitness=0;

  eval(sol);
  for(int i=0;i<5;i++){
    sum+=sol[i];
    sol[i]=0;
  }

  assert((int)(sol.fitness())==sum);
  eval(sol);
  assert((int)(sol.fitness())==0);

  sol[0]=1; 
  fitness=incr_eval(sol,fitness,0);
  assert((int)(fitness)==1);
  assert((int)(sol.fitness())==1);
	
  sol[2]=1;
  fitness=incr_eval(sol,fitness,2);
  assert((int)(fitness)==2);
  assert((int)(sol.fitness())==2);

  std::cout << "[t-moGPUOneMaxIncrEval] => OK" << std::endl;

  return EXIT_SUCCESS;
}

