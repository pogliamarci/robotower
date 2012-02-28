/***************************************************************************
                          command_list_debug.cpp  -  description
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

#include "command_list_debug.h"
#include <iostream>

void viewcdl(command_list * cdlist)
{

  command_list::iterator q;

  cout<<"\n------------------------- PRINTING COMMAND LIST ------------------------------\n";
  fflush(stdout);

  cout<<"\nLenght of command list = ";
  cout<<cdlist->size();

  int num = 0;

  for(q=cdlist->begin();q!=cdlist->end();q++)
    {
      cout<<"\ni = ";
      cout<<num;
      cout<<"\nCommand = ";
      cout<<(*q).first;
      cout<<"\nSetpoint = ";
      cout<<(*q).second->get_set_point();
      num++;
    }
  fflush(stdout);
}

