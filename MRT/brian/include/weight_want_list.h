/***************************************************************************
                          weight_want_list.h  -  description
                             -------------------
    begin                : Wed Sep 13 2000
    copyright            : (C) 2000 by Halva Giovanni & Giacomo
    email                : invehalv@airlab.elet.polimi.it
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/


//	This module declares the object used by others modules
//	to interface each others.
//
//      Status: TESTED

#ifndef weight_want_list_h
#define weight_want_list_h 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>

#ifdef DMALLOC

#include <dmalloc.h>

#endif


#include "weight_want_multimap.h"


//	Class that collects weight_want.
/**
 * This class collects and manages want weights. Methods for adding and searching were implemented.
 * @short List of want weights
 * @see weight_want
 * @see wanter
 * @see composer
 */
class weight_want_list : public weight_want_multimap
{
  public:
      /**
       * Constructor. It simply calls its parent's constructor.
       * @return It returns an empty weight want list.
       */
      weight_want_list();

      /**
       * Copy constructor. It simply calls its parent's copy constructor.
       * @param right Object to be copied
       * @return It returns a copy of the given list.
       */
      weight_want_list(const weight_want_list &right);

      /**
       * Destructor.
       * It does not delete objects it is pointing to!
       */
      virtual ~weight_want_list();


      /**
       * It looks into the list for a weight associated to with given behavior name.
       * @param name Behavior name to look for.
       * @return Weight of behavior if found, 0 otherwise.
       */
      virtual float get_weight (const char *name);

      //	Add a wieght_want object into the list. Return value
      //	not specified.
      /**
       * Add a pointer to a new weight to the list.
       * @param aweightwant Pointer to weight to add. 
       */
      virtual void add (weight_want *aweightwant);

};

#endif
