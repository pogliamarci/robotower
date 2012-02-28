/***************************************************************************
                          engine_objects.cpp  -  description
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

//	Description: This module contain abstract objects those
//	are used in the other engine module, such as Predicate,
//	Cando and Want Maker. The most important abstract
//	objectis the tree of operations.
//
//------------------------------------------------------------------------

// Engine_Objects
#include "engine_objects.h"

//*************************************************************************

// Class operation 

//************************************************************************

operation::operation()
{
}

operation::operation(const operation &right)
{
}

operation::~operation()
{
}

//************************************************************************

// Class not 

//************************************************************************

op_not::op_not(operation * apTerm)
{ 
  pTerm = apTerm;
}

op_not::op_not(const op_not &right)
{ }

op_not::~op_not()
{ }

float op_not::get_value(predicate_list *p, fuzzy_data_list *f)
{
  return (1 - (get_pTerm())->get_value(p,f));
}

float op_not::get_reliability(predicate_list *p, fuzzy_data_list *f)
{
  return ((get_pTerm())->get_reliability(p,f));
  // la reliability deve essere sempre la stessa
}

//************************************************************************

// Class and 

//************************************************************************

op_and::op_and(operation * apTerm_1,operation * apTerm_2)
{ 
  pTerm_1 = apTerm_1;
  pTerm_2 = apTerm_2;
}

op_and::op_and(const op_and &right)
{ }

op_and::~op_and()
{ }

float op_and::get_value(predicate_list *p,fuzzy_data_list *f)
{ 
  float dx =  get_pTerm_1()->get_value(p,f);
  float sx =  get_pTerm_2()->get_value(p,f);
  
  if ( dx > sx ) return sx;
  else return dx;
}

float op_and::get_reliability(predicate_list *p,fuzzy_data_list *f)
{ 
  float dx =  get_pTerm_1()->get_reliability(p,f);
  float sx =  get_pTerm_2()->get_reliability(p,f);
  
  if ( dx > sx ) return sx;
  else return dx;
}

//************************************************************************

// Class or 

//************************************************************************

op_or::op_or(operation * apTerm_1,operation * apTerm_2)
{ 
  pTerm_1 = apTerm_1;
  pTerm_2 = apTerm_2;
}

op_or::op_or(const op_or &right)
{ }

op_or::~op_or()
{ }

float op_or::get_value(predicate_list *p,fuzzy_data_list *f)
{ 
  float dx =  get_pTerm_1()->get_value(p,f);
  float sx =  get_pTerm_2()->get_value(p,f);
  
  if ( dx > sx ) return dx;
  else return sx;
}

float op_or::get_reliability(predicate_list *p,fuzzy_data_list *f)
{ 
  float dx =  get_pTerm_1()->get_reliability(p,f);
  float sx =  get_pTerm_2()->get_reliability(p,f);
  
  if ( dx > sx ) return dx;
  else return sx;
}

//***********************************************************************

// class data_node

//***********************************************************************

data_node::data_node()
{}

data_node::data_node(char* aname,char* alabel)
{
  name = (char *) malloc(strlen(aname)+1);
  strcpy(name,aname);

  label = (char *) malloc(strlen(alabel)+1);
  strcpy(label,alabel);
}

data_node::data_node(const data_node &right)
{}

data_node::~data_node()
{
  if (name!=NULL) free(name);
  if (label!=NULL) free(label);
}

float data_node::get_value(predicate_list *p,fuzzy_data_list *f)
{
  fuzzy_data * current = f->get_by_name_label(name,label);

  // modified in version 2.1 
  if (current != NULL) return(current->get_membership_value());
  else  return(0);
}

float data_node::get_reliability(predicate_list *p,fuzzy_data_list *f)
{
  fuzzy_data * current = f->get_by_name_label(name,label);
  
  // modified in version 2.1 
  if (current != NULL) return(current->get_reliability());
  else  return(1);
}

//***********************************************************************

// class predicate_node

//***********************************************************************

predicate_node::predicate_node()
{}

predicate_node::predicate_node(char* aname)
{
  name = (char *) malloc(strlen(aname)+1);
  strcpy(name,aname); 
}

predicate_node::predicate_node(const predicate_node &right)
{}

predicate_node::~predicate_node()
{
  if (name!=NULL) free(name);
}

float predicate_node::get_value(predicate_list *p,fuzzy_data_list *f)
{
  predicate * current = p->get(name);
  // modified in version 2.1 
  if (current != NULL) return(current->get_value());
  else  return(0);
}

float predicate_node::get_reliability(predicate_list *p,fuzzy_data_list *f)
{
  predicate * current = p->get(name);

  // modified in version 2.1 
  if (current != NULL) return(current->get_reliability());
  else  return(1);
}

//***********************************************************************

// Class aggregation_tree

//***********************************************************************
/*   E' il nodo pase da cui si sviluppa l'albero di nodi
     del tipo operation.
*/

aggregation_tree::aggregation_tree()
{
  name=NULL;
  pTerm=NULL;
}

aggregation_tree::aggregation_tree(char *aname, operation * apTerm)
{
  name = (char *) malloc(strlen(aname)+1);
  strcpy(name,aname);

  pTerm = apTerm;
}

aggregation_tree::~aggregation_tree()
{
 if (name!=NULL) free(name);
}

float aggregation_tree::get_value(predicate_list *p, fuzzy_data_list *f)
{
  return ((get_pTerm())->get_value(p,f));
}

float aggregation_tree::get_reliability(predicate_list *p, fuzzy_data_list *f)
{
  return ((get_pTerm())->get_reliability(p,f));
}

//------------------------------------------------------------------

//******************************************************************
//******************************************************************
// Class aggr_tree_parser
//******************************************************************

aggr_tree_parser::aggr_tree_parser(char * afilename)
{ 
  filename = (char *) malloc(strlen(afilename)+1);
  strcpy(filename,afilename);
} 
 
aggr_tree_parser::aggr_tree_parser(const aggr_tree_parser &right) 
{  
} 
 
 
aggr_tree_parser::~aggr_tree_parser() 
{ 
  if (filename!= NULL) free(filename);
} 
 
 
extern FILE *predin;
extern aggr_tree_multimap * parsefile(aggr_tree_multimap *anaggrmmap);

aggr_tree_multimap * aggr_tree_parser::get_aggr_trees (aggr_tree_multimap *anaggrmmap)
{
if ((predin=fopen(filename,"r"))!=NULL)
    {
      //if the file exists
      aggr_tree_multimap * ptm=parsefile(anaggrmmap);
      fclose(predin);
      return ptm;
    }
  else
    {
    // it it doesn't exist
    printf("FILE %s NOT FOUND",filename);
    return NULL;
    }

}; // End of get_aggr_trees
