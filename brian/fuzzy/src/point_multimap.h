/***************************************************************************
                          point_multimap.h  -  description
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


#ifndef point_multimap_h
#define point_multimap_h 1

#include "ord.h"
#include "point.h"

#include <stl.h>

#ifdef DMALLOC

#include <dmalloc.h>

#endif

/**
 * Instantiation of multimap template in order to create a point list
 * @see point_list
 */
typedef multimap< float, point *, ord > point_multimap;

#endif
