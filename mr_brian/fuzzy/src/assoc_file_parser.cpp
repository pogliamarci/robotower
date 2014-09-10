/***************************************************************************
                          assoc_file_parser.cpp  -  description
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
//      Module: assoc_file_parser
//	This module implements the classes used for
//	fuzzyfication and defuzzyfication of data.



#include "assoc_file_parser.h"

#include "math.h"


// Class assoc_file_parser 

assoc_file_parser::assoc_file_parser()
  : name(NULL)
{
}

assoc_file_parser::assoc_file_parser(const assoc_file_parser &right)
  : name(NULL)
{
  name=(char *) malloc(strlen(right.name)+1);
  strcpy(this->name,right.name);
}

assoc_file_parser::assoc_file_parser (char *file_name)
  : name(NULL)
{
  name=(char *) malloc(strlen(file_name)+1);
  strcpy(name,file_name);
}


assoc_file_parser::~assoc_file_parser()
{
  if (name != NULL) free(name);
}

//## Other Operations (implementation)
extern FILE *associn;
extern association_list * assoc_parser();

association_list * assoc_file_parser::read_associations ()
{
  if ((associn=fopen(name,"r"))!=NULL)
    {
      //if the file exists
      association_list * al=assoc_parser();
      fclose(associn);
      return al;
    }
  else
    {
    //  it doesn't exist
    printf("FILE %s NOT FOUND\n",name);
    return NULL;
    }
}
