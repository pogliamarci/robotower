/***************************************************************************
                          command.cpp  -  description
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



#include "command.h"


command::command()
      : set_point(0)
{
  label=NULL;
}

command::command(const command& right)
{
  this->label=(char *) malloc(strlen(right.label)+1);
  strcpy(this->label,right.label);
  this->set_point=right.set_point;
}

command::command (const char *alabel, float aset_point)
      : set_point(0)
{
  label=(char *) malloc(strlen(alabel)+1);
  strcpy(label,alabel);
  set_point=aset_point;
}

command::~command()
{
  if (label!=NULL) free (label);
}
