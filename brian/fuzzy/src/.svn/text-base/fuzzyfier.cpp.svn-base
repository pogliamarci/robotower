/***************************************************************************
                          fuzzyfier.cpp  -  description
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
//      Module: fuzzyfier
//	This module implements the classes used for
//	fuzzyfication and defuzzyfication of data.



#include "fuzzyfier.h"

#include <math.h>


// Class fuzzyfier 

fuzzyfier::fuzzyfier()
  :fuzzy_crisp_rel()
{
}

fuzzyfier::fuzzyfier(const fuzzyfier &right)
  :fuzzy_crisp_rel(right)
{
}

fuzzyfier::fuzzyfier (shape_file_parser *shapefileparser, assoc_file_parser *assocfileparser)
  :fuzzy_crisp_rel(shapefileparser,assocfileparser)
{
}


fuzzyfier::~fuzzyfier()
{
}

//## Other Operations (implementation)

fuzzy_data_list * fuzzyfier::fuzzyfy (crisp_data_list *datalist)
{
  shapes_list * sl = get_shapelist();
  association_list * al = get_assoc();

  fuzzy_data_list * list = new fuzzy_data_list();

  association_list::iterator i = al->begin();            
  while (i!=al->end())
    {
      const char * name = (*i).second->get_label();              //get from the association the name of the crisp data to fuzzyfy
      crisp_data * data = datalist->get_by_name(name);     //get the crisp data from the list
   
      if (data != NULL)
	{
	  const char * tempshape = (*i).second->get_shape();     //get the name of the shape from the association    
	  shape * curr = sl->get_shape(tempshape);         //get the shape from the shapelist

	  float cv = data->get_value();                  

	  shape::iterator j = curr->begin();
	  while (j!=curr->end())
	    {
	      const char * templabel = (*j).second->get_label(); //get the label from the shape
	      // modified in version 2.1
	      float mv = (*j).second->get_membership_value(cv);
	      float re = data->get_reliability();

	      if (mv != 0 || re != 1) list->add(new fuzzy_data(name,templabel,mv,re));
	      j++;
	    }
	}
      i++;
    }
  return (list);
}

