/***************************************************************************
                          preacher.cpp  -  description
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

//	Description: This module at strart up create the
//	Predicate tree and its list by the predicate parser class.
//	At the runtime it evalues the predicates tree by the
//	fuzzy data list and creates a predicate list.
//
//----------------------------------------------------------------
//----------------------------------------------------------------


// Preacher
#include "preacher.h"

//------------------------------------------------------------------

//******************************************************************
//******************************************************************
// Class prd_tree_map  
//******************************************************************


prd_tree_map::prd_tree_map() 
  :aggr_tree_multimap()
{ 
} 
 
prd_tree_map::prd_tree_map(const prd_tree_map &right) 
  :aggr_tree_multimap(right)
{ 
} 

prd_tree_map::~prd_tree_map() 
{ 
  for_each(begin(),end(),destroy_object<aggregation_tree> ());
} 
  

//---------------------------------------------------------------
//## Other Operations (implementation)
//---------------------------------------------------------------

predicate_list * prd_tree_map::evalue (fuzzy_data_list * fuzzy_lst)
{
  // begin prd_tree_map::evalue
  predicate_list * prd_lst = NULL;
  iterator i;

  // Creo la lista di predicati da ritornare
  prd_lst = new predicate_list();

  //printf ("------------------------- EVALUE ------------------------------\n");
    
  for (i = begin(); i != end(); i++) {
    // Prendo il puntatore all'albero del predicato.
    aggregation_tree *prd_tree = (*i).second;
  
    char* name = prd_tree->get_name(); 
    float value = prd_tree->get_value(prd_lst,fuzzy_lst);
    float reliability = prd_tree->get_reliability(prd_lst,fuzzy_lst);
      
   // modified version 2.1
   // Predicati con reliability 1 e value 0 (certamente falsi) non vengono aggiunti
  	if (value != 0 || reliability != 1)
      {
			predicate * current = new predicate (name,value,reliability);
   		prd_lst->add(current);
      }
    //================================================================
    //================================================================
    // Per la visualizzazione del contenuto della lista

//     printf ("Predicato : %s  Value : %f  Reliability : %f \n \n",
//     name, value, reliability);
    //================================================================
    //================================================================
	delete[] name;
    
  };// End for

  return prd_lst;
  // end prd_tree_map::evalue%
}
//----------------------------------------------------------------

preacher::preacher()
{
    pred_parser=NULL;
    predicates=NULL;
}

preacher::preacher(aggr_tree_parser * apredicateparser)
{
    pred_parser=apredicateparser;
    predicates=new prd_tree_map();
    pred_parser->get_aggr_trees(predicates);
}

preacher::~preacher()
{
    delete pred_parser;
    delete predicates;
}

predicate_list *  preacher::preach(fuzzy_data_list * fuzzy_lst)
{
    return predicates->evalue(fuzzy_lst);
}
