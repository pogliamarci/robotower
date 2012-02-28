/***************************************************************************
                          fuzzy_crisp_rel.h  -  description
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
//	Specifies the classes used to fuzzyfy and defuzzyfy data


#ifndef fuzzy_crisp_rel_h
#define fuzzy_crisp_rel_h 1

#ifdef DMALLOC

#include <dmalloc.h>

#endif

#include "shapes_list.h"
#include "association_list.h"
#include "shape_file_parser.h"
#include "assoc_file_parser.h"

/**
* This abstract class identifies the relationship between fuzzy world and crisp world.
* @short abstract representation between crisp and fuzzy 
*/

class fuzzy_crisp_rel 
{
 public:
  /**
   * Constructor generated by the UML software used to build Brian. It return an empty fuzzy crisp rel.
   */
  fuzzy_crisp_rel();

  /**
   * Copy constructor generated by the UML software used to build Brian. It return a copy of the fuzzy crisp rel.
   */
  fuzzy_crisp_rel(const fuzzy_crisp_rel &right);

  /*
   * Constructor that build a fuzzy crisp rel with associations and shapes read by the parsers.
   * @param shapefileparser pointers to an object of kind shape_file_parser containing shapes
   * @param assocfileparser pointers to an object of kind assoc_file_parser containing associations 
   */
  fuzzy_crisp_rel (shape_file_parser *shapefileparser, assoc_file_parser *assocfileparser);

  /**	
   * Destructor destroy a shape fuzzy crisp rel: free the shapes list and the association list read at startup.
   */
  ~fuzzy_crisp_rel();
  
  /**
   * Method that return a pointer to the shape list contained into the relation. It point to the original list so it must not be destroyed.
   * @return a pointer to a shape list
   */ 
  shapes_list * get_shapelist ();
  /**
   * Set the shape list into the relation.
   * @param value is the new shape list
   */  
  void set_shapelist (shapes_list * value);

  /**
   * Method that return a pointer to the association list contained into the relationship. It point to the original list so it must not be destroyed.
   * @return a pointer to an association list
   */ 
  association_list * get_assoc ();
  /**
   * Set the association list into the relation.
   * @param value is the new association list
   */  
  void set_assoc (association_list * value);

 private:
      // Data Members for Class Attributes

      shapes_list *shapelist;

      association_list *assoc;
};


// Class fuzzy_crisp_rel 

inline shapes_list * fuzzy_crisp_rel::get_shapelist ()
{
  return shapelist;
}

inline void fuzzy_crisp_rel::set_shapelist (shapes_list * value)
{
  shapelist = value;
}

inline association_list * fuzzy_crisp_rel::get_assoc ()
{
  return assoc;
}

inline void fuzzy_crisp_rel::set_assoc (association_list * value)
{
  delete assoc;
  assoc = value;
}


#endif
