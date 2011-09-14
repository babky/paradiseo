/*
 <moGPUPermutationVector.h>
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

#ifndef __moGPUPermutationVector_H_
#define __moGPUPermutationVector_H_

#include <GPUType/moGPUIntVector.h>

/**
 * Implementation of Permutation vector representation on GPU.
 */

template<class Fitness>

class moGPUPermutationVector: public moGPUIntVector<Fitness> {

public:

	using moGPUIntVector<Fitness>::vect;
	using moGPUIntVector<Fitness>::N;

	/**
	 * Default constructor.
	 */

	moGPUPermutationVector() :
		moGPUIntVector<Fitness> () {

	}

	/**
	 *Constructor.
	 *@param _size The solution size.
	 */

	moGPUPermutationVector(unsigned _size) :
		moGPUIntVector<Fitness> (_size) {
		create();
	}
	/**
	 *Initializer of random permuatation vector.
	 */
	void create() {

		unsigned random;
		int tmp;
		for (unsigned i = 0; i < N; i++)
			vect[i] = i;
		// we want a random permutation so we shuffle
		for (unsigned i = 0; i < N; i++) {
			random = rng.rand() % (N - i) + i;
			tmp = vect[i];
			vect[i] = vect[random];
			vect[random] = tmp;
		}
	}


};

#endif
