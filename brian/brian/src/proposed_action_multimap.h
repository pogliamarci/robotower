/***************************************************************************
                          proposed_action_multimap.h  -  description
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

#ifndef proposed_action_multimap_h
#define proposed_action_multimap_h 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>

#ifdef DMALLOC

#include <dmalloc.h>

#endif


#include "proposed_action.h"

#include <ltstr.h> //fuzzy/include

#include <stl.h>


/**
 * Instantiation of multimap template in order to create a proposed action list
 * @see proposed_action_list
 */
typedef multimap<const char *, proposed_action *, ltstr > proposed_action_multimap;

#endif
