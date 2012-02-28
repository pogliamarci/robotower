/***************************************************************************
                          fuzzy_data.cpp  -  description
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
//	This module implements fuzzy_data.h


#include "fuzzy_data.h"

// Class fuzzy_data 

fuzzy_data::fuzzy_data()
      : data(), membership_value(0)  
{
  label=NULL;
}

fuzzy_data::fuzzy_data(const fuzzy_data& right)
  :data(right)
{
  this->label=(char *) malloc(strlen(right.label)+1);
  strcpy(this->label,right.label);
  this->membership_value=right.membership_value;
}

fuzzy_data::fuzzy_data (const char *aname, const char *alabel, float am_f)
  : data(aname), membership_value(0)
{
  label=(char *) malloc(strlen(alabel)+1);
  strcpy(label,alabel);
  membership_value=am_f;
}

fuzzy_data::fuzzy_data (const char *aname, const char *alabel, float am_f, float areliability)
      : data(aname,areliability), membership_value(0)
{
  label=(char *) malloc(strlen(alabel)+1);
  strcpy(label,alabel);
  membership_value=am_f;
}


fuzzy_data::~fuzzy_data()
{
  if (label!= NULL) free(label);
}

