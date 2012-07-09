/***************************************************************************
                          association_set_multimap.h  -  description
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


#ifndef association_set_multimap_h
#define association_set_multimap_h 1

#include "ltstr.h"
#include "association.h"

#include <stl.h>

#ifdef DMALLOC

#include <dmalloc.h>

#endif
/**
 * Instantiation of multimap template in order to create an association list
 * @see association_list
 */
typedef multimap<const char*,association*,ltstr > association_set_multimap;

#endif
