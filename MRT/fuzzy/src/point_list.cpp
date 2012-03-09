/***************************************************************************
                          point_list.cpp  -  description
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
//      Module: point_list
//	This module implements the classes used for
//	fuzzyfication and defuzzyfication of data.


#include "point_list.h"

#include <math.h>

// Class point_list 

point_list::point_list()
  :point_multimap()
{
}

point_list::point_list(const point_list &right)
  :point_multimap(right)
{
}


point_list::~point_list()
{
  for_each(this->begin(), this->end(), destroy_point_list<point>());
}

//## Other Operations (implementation)

void point_list::add_point (point * apoint)
{
  insert(pair<float, point *>(apoint->get_xa(),apoint));
}


