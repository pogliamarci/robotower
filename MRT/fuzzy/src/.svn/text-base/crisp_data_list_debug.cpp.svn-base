/***************************************************************************
                          crisp_data_list_debug.cpp  -  description
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
//	This module implements crisp_data_list_debug.h

#include "crisp_data_list_debug.h"
#include <iostream>

void viewcdl(crisp_data_list * cdl)
{
  crisp_data_list::iterator w;

  cout<<"\n------------------------- PRINTING CRISP DATA LIST ---------------------------\n";
  fflush(stdout);

  cout<<"\nLengh of crisp data list = ";
  cout<<cdl->size();

  int num = 0;

  for(w=cdl->begin();w!=cdl->end();w++)
    {
      cout<<"\ni = ";
      cout<<num;
      cout<<"\nName = ";
      cout<<(*w).second->get_name();
      cout<<"\nValue = ";
      cout<<(*w).second->get_value();
      cout<<"\nReliability = ";
      cout<<(*w).second->get_reliability();
      num++;
    }
   fflush(stdout);
}
