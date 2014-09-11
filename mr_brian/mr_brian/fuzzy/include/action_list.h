/***************************************************************************
                          action_list.h  -  description
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

#ifndef action_list_h
#define action_list_h 1

#include "action_multimap.h"

#ifdef DMALLOC

#include <dmalloc.h>

#endif


/**
 * This class collects and manages actions. Methods for adding and searching were implemented.
 * @short List of actions
 * @see action
 * @see composer
 * @see defuzzyfier
 */
class action_list : public action_multimap
{
  public:
      /**
       * Constructor. It simply calls its parent's constructor.
       * @return It returns an empty action list.
       */
      action_list();

      /**
       * Copy constructor. It simply calls its parent's copy constructor.
       * @param right Object to be copied
       * @return It returns a copy of the given list.
       */
      action_list(const action_list &right);

      /**
       * Destructor.
       * It does not delete objects it is pointing to!
       */
      virtual ~action_list();


      /**
       * This method is used to search some actions in a list. Since there can
       * be more than one action with the same name, all of them are returned
       * in another action list. When you finished using the returned object,
       * you must delete it but not the pointed objects!
       *
       * @param name Name too look for
       * @return an action list whose elements have all the given name. The list is indexed by label. If no element is found, an empty list is returned.
       */
      virtual action_list * get_by_name (const char *name);

      /**
       * It looks into the list for an action with given name and label
       * @param name Action name to look for.
       * @param label Action label to look for.
       * @return a pointer to the desired action, NULL otherwise.
       */
      virtual action * get_by_name_label (const char *name, const char *label);

      //	Add an action object into the list. Retrun value: not
      //	specified.
      /**
       * Add a pointer to a new action in the list.
       * @param anaction Pointer to the action to add.
       */
      virtual void add (action *anaction);
};

#endif
