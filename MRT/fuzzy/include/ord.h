/***************************************************************************
                          ord.h  -  description
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

#ifndef ord_h
#define ord_h 1

#ifdef DMALLOC

#include <dmalloc.h>

#endif

/**
* this structure is used by multimaps as ordinated insertion method
*/
struct ord {
  bool operator() (float s1,float s2) const
    {
      return s1<s2;
    }
};

#endif
