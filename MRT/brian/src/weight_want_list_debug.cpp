/***************************************************************************
                          weight_want_list_debug.cpp  -  description
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
//	This module implements the objects used by other object
//	to interface each others.


#include "weight_want_list_debug.h"
#include <iostream>

void viewwwl(weight_want_list * wwl)
{
  int num = 0;

  weight_want_list::iterator w;

  cout<<"\n------------------------- PRINTING WEIGHT WANT LIST ---------------------------\n";
  cout<<"\nLenght of weight want list = ";
  cout<<wwl->size();
  fflush(stdout);

  for(w=wwl->begin();w!=wwl->end();w++)
    {
      cout<<"\ni = ";
      cout<<num;
      cout<<"\nName = ";
      cout<<(*w).second->get_name();
      cout<<"\nValue = ";
      cout<<(*w).second->get_value();
      num++;
      fflush(stdout);
    }
}

