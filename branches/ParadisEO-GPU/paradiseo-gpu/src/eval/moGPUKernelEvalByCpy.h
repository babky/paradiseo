/*
 <moGPUKernelEvalByCpy.h>
 Copyright (C) DOLPHIN Project-Team, INRIA Lille - Nord Europe, 2006-2010

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

#ifndef __moGPUKernelEvalByCpy_H
#define __moGPUKernelEvalByCpy_H
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * The kernel function called from the host and executed in device to compute all neighbors fitness at one time
 * without mapping, each thread id compute one fitness by copying localy the solution
 * @param _eval how to evaluate each neighbor
 * @param _solution the representation of solution( vector of int,float....)
 * @param _allFitness the array of Fitness to save all neighbors fitness
 * @param _fitness the current solution fitness
 * @param _neighborhoodsize the size of the neighborhood
 */

template<class T, class Fitness, class Eval>

__global__ void moGPUKernelEvalByCpy(Eval _eval, T * _solution, Fitness* _allFitness,
		Fitness _fitness, unsigned _neighborhoodsize) {

	// The thread identifier within a grid block's
	int id = blockIdx.x * blockDim.x + threadIdx.x;
	// array to save index to be changed
	unsigned index[2];
	T sol_tmp[SIZE];
	// In this representation each id identify one and only one neighbor in neighborhood
	if (id < _neighborhoodsize) {
		for(unsigned i=0;i<SIZE;i++)
		sol_tmp[i]=_solution[i];
		//Change the id'th element of solution
		index[0]=id;
		index[1]=_neighborhoodsize;
		//Compute fitness for id'th neighbor
		_allFitness[id] = _eval(sol_tmp,_fitness,index);
	}
}

#endif