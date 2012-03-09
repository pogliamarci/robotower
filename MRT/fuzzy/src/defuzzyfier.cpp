/***************************************************************************
                          defuzzyfier.cpp  -  description
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
//      Module: defuzzyfier
//	This module implements the classes used for
//	fuzzyfication and defuzzyfication of data.


#include "defuzzyfier.h"

#include "math.h"

// Class defuzzyfier 

defuzzyfier::defuzzyfier()
  :fuzzy_crisp_rel()
{
}

defuzzyfier::defuzzyfier(const defuzzyfier &right)
  :fuzzy_crisp_rel(right)
{
}

defuzzyfier::defuzzyfier (shape_file_parser *shapefileparser, assoc_file_parser *assocfileparser)
  :fuzzy_crisp_rel(shapefileparser,assocfileparser)
{
}


defuzzyfier::~defuzzyfier()
{
}

