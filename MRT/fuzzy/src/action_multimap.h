/***************************************************************************
                          action_multimap.h  -  description
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

#ifndef action_multimap_h
#define action_multimap_h 1

#include "action.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>

#include <ltstr.h>

#include <stl.h>

#ifdef DMALLOC

#include <dmalloc.h>

#endif

//	Abstract class that represents a multimap of actions.
/**
 * Instantiation of multimap template in order to create an action list
 * @see action_list
 */
typedef multimap<const char *, action *, ltstr > action_multimap;

#endif
