/*
 <moCudaObject.h>
 Copyright (C) DOLPHIN Project-Team, INRIA Lille - Nord Europe, 2006-2010

 Boufaras Karima, Thé Van Luong

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

#ifndef _moCudaObject_H_
#define _moCudaObject_H_

// CUDA includes
#include <cutil.h>
#include <memory/moCudaAllocator.h>
#include <memory/moCudaDesallocator.h>
#include <memory/moCudaCopy.h>

/**
 *  class of managment of data on GPU global memory (allocation,desallocation & copy)
 */

class moCudaObject {

public:

	/**
	 *Copy data from CPU memory to GPU global memory (default copy)
	 *@param _data the data to allocate on GPU
	 *@param _dataTocpy the data to copy from CPU memory to _data on GPU memory
	 *@param _dataSize the size of data to copy
	 */
	template<typename T>
	void memCopy(T* & _data, T * & _dataTocpy, unsigned _dataSize) {
		malloc(_data, _dataSize);
		copy(_data, _dataTocpy, _dataSize);
	}

	/**
	 *Copy device data from GPU global memory to global variable declared in device
	 *@param _dev_data the device global variable
	 *@param _dataTocpy the data to copy GPU global memory to GPU global variable
	 */
	template<typename T>
	void memCopyGlobalVariable(T* & _dev_data, T * & _dataTocpy) {
		copy(_dev_data, _dataTocpy);
	}

	/**
	 *Desallocate data on GPU global memory
	 *@param _data the data to desallocate from GPU global memory
	 */
	template<typename T>
	void memFree(T* & _data) {
		free(_data);
	}

public:

	moCudaAllocator malloc;
	moCudaCopy copy;
	moCudaDesallocator free;

};

#endif
