/***************************************************************************
                          fuzzy_set_multimap.h  -  description
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


#ifndef fuzzy_set_multimap_h
#define fuzzy_set_multimap_h 1

#include "fuzzy_set.h"
#include "ltstr.h"

#include <stl.h>

#ifdef DMALLOC

#include <dmalloc.h>

#endif

/**
 * Instantiation of multimap template in order to create a shape
 * @see shape
 */
typedef multimap< const char*,fuzzy_set*,ltstr> fuzz_set_multimap;

#endif
