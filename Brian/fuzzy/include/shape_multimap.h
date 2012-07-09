/***************************************************************************
                          shape_multimap.h  -  description
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

#ifndef shape_multimap_h
#define shape_multimap_h 1

#include "ltstr.h"
#include "shape.h"

#include <stl.h>

#ifdef DMALLOC

#include <dmalloc.h>

#endif


/**
 * Instantiation of multimap template in order to create a shape list
 * @see shape_list
 */
typedef multimap<const char*,shape*,ltstr > shape_multimap;

#endif
