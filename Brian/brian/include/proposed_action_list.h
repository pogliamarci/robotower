/***************************************************************************
                          proposed_action_list.h  -  description
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

#ifndef proposed_action_list_h
#define proposed_action_list_h 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>

#ifdef DMALLOC

#include <dmalloc.h>

#endif

#include "proposed_action_multimap.h"



/**
 * This class collects and manages proposed actions. Methods for adding and searching were implemented.
 * @short List of proposed actions
 * @see proposed_action
 * @see behavior
 * @see behavior_engine
 * @see composer
 */
class proposed_action_list: public  proposed_action_multimap
{
  public:
      /**
       * Constructor. It simply calls its parent's constructor.
       * @return It returns an empty proposed list.
       */
      proposed_action_list();

      /**
       * Copy constructor. It simply calls its parent's copy constructor.
       * @param right Object to be copied
       * @return It returns a copy of the given list.
       */
      proposed_action_list(const proposed_action_list &right);

      /**
       * Destructor.
       * It does not delete objects it is pointing to!
       */
      virtual ~proposed_action_list();

      /**
       * This method is used to search some proposed actions in a list. Since there can
       * be more than one action with the same name, all of them are returned
       * in another proposed action list. When you finished using the returned object,
       * you must delete it but not the pointed objects!
       *
       * @param name Name too look for
       * @return an action list whose elements have all the given name. The list is indexed by label. If no element is found, an empty list is returned.
       */
      virtual proposed_action_list * get_by_name (const char *name);

      /**
       * It looks into the list for a proposed action with given name and label
       * @param name Action name to look for.
       * @param label Action label to look for.
       * @return a pointer to the desired action, NULL otherwise.
       */
      virtual proposed_action_list * get_by_name_label (const char *name, const char *label);

      /**
       * Add a pointer to a new proposed action in the list.
       * @param anaction Pointer to the proposed action to add.
       */
      virtual void add (proposed_action *anaction);

      /**
       * Delete all actions whose name is name and label is label
       */
      virtual void del (const char *name, const char *label);

      /**
       * Delete all actions whose name is name and label is label and
       * are not been proposed by the behavior behavior
       */
      virtual void del (const char *name, const char *label, const char *behavior);


};

#endif
