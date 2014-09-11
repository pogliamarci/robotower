/***************************************************************************
                          shape_file_parser.cpp  -  description
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
//      Module: shape_file_parser
//	This module implements the classes used for
//	fuzzyfication and defuzzyfication of data.


#include "shape_file_parser.h"

#include <math.h>


// Class shape_file_parser 

// Class shape_file_parser 

shape_file_parser::shape_file_parser()
  : name(NULL)
{
}

shape_file_parser::shape_file_parser(const shape_file_parser &right)
  : name(NULL)
{
  name=(char *) malloc(strlen(right.name)+1);
  strcpy(this->name,right.name);
}

shape_file_parser::shape_file_parser (char *file_name)
  : name(NULL)
{
  name=(char *) malloc(strlen(file_name)+1);
  strcpy(name,file_name);
}


shape_file_parser::~shape_file_parser()
{
  if (name != NULL) free(name);
}

//## Other Operations (implementation)
extern FILE *shapein;
extern shapes_list * shape_parser();

shapes_list * shape_file_parser::read_shapelist ()
{
  if ((shapein=fopen(name,"r"))!=NULL)
    {
      //if the file exists
      shapes_list * sl=shape_parser();
      fclose(shapein);
      return sl;
    }
  else
    {
    //  it doesn't exist
    printf("FILE %s NOT FOUND\n",name);
    return NULL;
    }
}

