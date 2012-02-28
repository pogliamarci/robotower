/***************************************************************************
                          action.cpp  -  description
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




#include "action.h"

// Class action

action::action()
      : membership_value(0)
{
  name=NULL;
  label=NULL;
}

action::action(const action& right)
{
  this->name=(char *) malloc(strlen(right.name)+1);
  strcpy(this->name,right.name);
  this->label=(char *) malloc(strlen(right.label)+1);
  strcpy(this->label,right.label);
  this->membership_value=right.membership_value;
}

action::action (const char *aname, const char *alabel)
       :membership_value(0)
{
  name=(char *) malloc(strlen(aname)+1);
  strcpy(name,aname);
  label=(char *) malloc(strlen(alabel)+1);
  strcpy(label,alabel);
}

action::action (const char *aname, const char *alabel, float am_f)
{
  name=(char *) malloc(strlen(aname)+1);
  strcpy(name,aname);
  label=(char *) malloc(strlen(alabel)+1);
  strcpy(label,alabel);
  membership_value=am_f;
}

//SM: costruttore con reliability_value
action::action (const char *aname, const char *alabel, float am_f, float arel)
{
  name=(char *) malloc(strlen(aname)+1);
  strcpy(name,aname);
  label=(char *) malloc(strlen(alabel)+1);
  strcpy(label,alabel);
  membership_value=am_f;
  reliability_value=arel;
}




action::~action()
{
  if (name!=NULL) free(name);
  if (label!=NULL) free(label);
}
