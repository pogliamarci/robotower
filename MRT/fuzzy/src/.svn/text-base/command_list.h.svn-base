/***************************************************************************
                          command_list.h  -  description
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


#ifndef command_list_h
#define command_list_h 1

#include "command_multimap.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>

#ifdef DMALLOC

#include <dmalloc.h>

#endif

/**
 * This class collects and manages commands. Methods for adding and searching were implemented.
 * @short List of command
 * @see command
 * @see defuzzyfier
 */
class command_list : public command_multimap 
{
  public:
      /**
       * Constructor. It simply calls its parent's constructor.
       * @return It returns an empty command list.
       */
      command_list();

      /**
       * Copy constructor. It simply calls its parent's copy constructor.
       * @param right Object to be copied
       * @return It returns a copy of the given command list.
       */
      command_list(const command_list &right);

      /**
       * Destructor.
       * It does not delete objects it is pointing to!
       */
      virtual ~command_list();

      /**
       * It looks in the list for a command with given label.
       * @param label Label of the command to look for.
       * @return It returns a pointer to the desired object, or NULL if not found.
       */
      virtual command * get_command (const char *label);

      /**
       * Add a pointer to a new command in the list.
       * @param acommand Pointer to the command to add.
       */
      virtual void add (command *acommand);
};

#endif
