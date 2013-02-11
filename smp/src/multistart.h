/*
  <multistart.h>
  Copyright (C) DOLPHIN Project-Team, INRIA Lille - Nord Europe, 2006-2013

  Clive Canape

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

#ifndef MULTISTART_H
#define MULTISTART_H

#include <vector>
#include <thread>
#include <mutex>

std::mutex m_multistart;

//! \class Multistart
//! \brief Parallel wrapper for local search
//! \tparam LS local search
//! \tparam T type of invidudals
//! \version 0.1
//! \date 2013
//! \author Clive Canape
template < class LS, typename T >
class Multistart
{
 public:

  //! \brief Constructor
  //! \tparam Args templates of the LS's arguments
  //! \param _nbThreads number of threads
  //! \param args LS's arguments
  template <class... Args>
  Multistart(unsigned _nbThreads, Args&... args)
    :nbThreads(_nbThreads)
  {
    ls = new LS (args...);
  }

  //! \brief Constructor
  //! \tparam Args templates of the LS's arguments
  //! \param args LS's arguments
  template <class... Args>
  Multistart(Args&... args)
    :Multistart(std::thread::hardware_concurrency(), args...)
  {}

  //! \brief Destructor
  ~Multistart()
  {
    delete ls;
  }

  //! \brief Run the procesing
  //! \param _pop population that is treated
  void operator()(std::vector<T> & _pop)
  {
	  // To create the threads
	  for(unsigned i = 0; i < nbThreads; i++)
	  {
		  threads.push_back(std::thread ([](const LS _ls, std::vector<T> & _pop,unsigned & _counter)
				  {
		  	  	  	  bool stop = false;
		  	  	  	  unsigned local;
		  	  	  	  while(!stop)
		  	  	  	  {
		  	  	  		  LS ls(_ls);
		  	  	  		  // To get the number of the indivudal which is going to threat
		  	  	  		  std::unique_lock<std::mutex> lock(m_multistart);
		  	  	  		  local = _counter;
		  	  	  		  _counter++;
		  	  	  		  lock.unlock();

		  	  	  		  // To apply the local search on the individual
		  	  	  		  if(local < _pop.size())
		  	  	  		  {
		  	  	  			  try
		  	  	  			  {
		  	  	  				  ls(_pop[local]);
		  	  	  			  }
		  	  	  			  catch(std::exception& e) {} //! \bug FIXME problem with ILS
		  	  	  		  }
		  	  	  		  else
		  	  	  			  stop = true;
		  	  	  	  }
				  },
				  *ls,
				  std::ref(_pop),
				  std::ref(counter)));
	  }

	  // Wait every threads
	  for(auto & i : threads)
		  i.join();
  }

 private:

  //! \param nbThreads number of threads
  unsigned nbThreads;

  //! \param ls local search that is used
  LS * ls;

  //! \param counter global counter
  unsigned counter = 0;

  //! \param threads list of threads
  std::vector < std::thread > threads;
};

#endif
