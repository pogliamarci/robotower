/***************************************************************************
                           predicate_list.h  -  description
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

#ifndef predicate_list_h
#define predicate_list_h 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>

#ifdef DMALLOC

#include <dmalloc.h>

#endif

#include "predicate_multimap.h"



//	Class that collects predicates to be evaluated
/**
 * This class collects and manages predicates and active behaviors. Methods for adding and searching were implemented.
 * @short List of predicates and active behaviors.
 * @see predicate
 * @see prd_tree_map
 * @see behavior_engine
 * @see cando_tree_map
 * @see wanter
 */
class predicate_list : public predicate_multimap
{
  public:
      /**
       * Constructor. It simply calls its parent's constructor.
       * @return It returns an empty list.
       */
      predicate_list();

      /**
       * Copy constructor. It simply calls its parent's copy constructor.
       * @param right Object to be copied
       * @return It returns a copy of the given list.
       */
      predicate_list(const predicate_list &right);

      /**
       * Destructor.
       * It does not delete objects it is pointing to!
       */
      ~predicate_list();

      //	Returns the predicate object whose name is equal to the
      //	parameter.
      /**
       * It looks in the list for an object with given name.
       * @param name Name of the object to look for.
       * @return It returns a pointer to the desired object, or NULL if not found.
       */
      predicate * get (const char *name);

      //	Add a predicate object into the list. Return value:
      //	not specified.
      /**
       * Add a pointer to a new object in the list.
       * @param apredicate Pointer to the object to add.
       */
      void add (predicate *apredicate);
      predicate_list  operator = (predicate_list &p);

      /**
       * Delete all the predicates related to ProposedActions
       */
      void delete_proposed ();
};


#endif
