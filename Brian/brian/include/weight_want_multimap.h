/***************************************************************************
                          weight_want_multimap.h  -  description
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

#ifndef weight_want_multimap_h
#define weight_want_multimap_h 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>

#ifdef DMALLOC

#include <dmalloc.h>

#endif


// stl
#include <stl.h>

#include "weight_want.h"

#include <ltstr.h> //fuzzy/include


/**
 * Instantiation of multimap template in order to create a want list
 * @see weight_want_list
 */
typedef multimap<const char *, weight_want *, ltstr > weight_want_multimap;

#endif
