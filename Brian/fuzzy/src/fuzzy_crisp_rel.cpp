/***************************************************************************
                          fuzzy_crisp_rel.cpp  -  description
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
//      Module: fuzzy_crisp_rel
//	This module implements the classes used for
//	fuzzyfication and defuzzyfication of data.




#include "fuzzy_crisp_rel.h"

#include <math.h>


// Class fuzzy_crisp_rel 

fuzzy_crisp_rel::fuzzy_crisp_rel()
{
  shapelist=NULL;
  assoc=NULL;
}

fuzzy_crisp_rel::fuzzy_crisp_rel(const fuzzy_crisp_rel &right)
{
  this->shapelist=right.shapelist;
  this->assoc=right.assoc;
}

fuzzy_crisp_rel::fuzzy_crisp_rel (shape_file_parser *shapefileparser, assoc_file_parser *assocfileparser)
{
  shapelist=shapefileparser->read_shapelist();
  assoc=assocfileparser->read_associations();
  delete shapefileparser;
  delete assocfileparser;
}


fuzzy_crisp_rel::~fuzzy_crisp_rel()
{
  if (shapelist != NULL) delete shapelist;
  if (assoc != NULL) delete assoc;
}


