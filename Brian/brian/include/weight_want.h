/***************************************************************************
                          weight_want.h  -  description
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


//	This module declares the object used by others modules
//	to interface each others.
//
//      Status: TESTED

#ifndef weight_want_h
#define weight_want_h 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>

#ifdef DMALLOC

#include <dmalloc.h>

#endif



//	Class that represents the weight of each want condition.
/**
 * Weights coming from wanter user by composer to tune proposed actions.
 * @short Weight of want condition
 * @see wanter
 * @see composer
 * @see weight_want_list
 */
class weight_want
{
  public:

      /**
       * Empty constructor
       * @return It returns an empty weight
       */
      weight_want();

      /**
       * Copy constructor
       * @param right The object to be copied
       * @return It returns a copy of the given weight
       */
      weight_want(const weight_want &right);

      //	Constructor: "aname" represent the name of the want
      //	condition, and "avalue" the associated weight.
      /**
       * Constructor
       * @param aname Behavior name the weight refers to.
       * @param avalue Value of want condition.
       * @return It returns an object with given name and value
       */
      weight_want (const char *aname, float avalue);

      /**
       * Destructor.
       */
      virtual ~weight_want();

      /**
       * Method to retrieve weight value.
       * @return Weight value.
       */
      float get_value ();

      /**
       * Method to set weight value.
       * @param value New weight value.
       */
      void set_value (float Value);

      //	Name of the want condition.
      /**
       * Method to retrieve related behavior name.
       * @return A pointer to behavior name.
       */
      const char * get_name ();
      
      /**
       * Method to set behavior name.
       * @param value Pointer to new behavior name. It must be freed after call.
       */
      void set_name (const char * value);

  private:
    // Data Members for Class Attributes
      float value;
      char *name;
};

// Class weight_want

inline float weight_want::get_value ()
{
  return value;
}

inline void weight_want::set_value (float Value)
{
  value = Value;
}

inline const char * weight_want::get_name ()
{
  return name;
}

inline void weight_want::set_name (const char * value)
{
  if (name!=NULL) free(name);
  name=(char *) malloc(strlen(value)+1);
  strcpy(name,value);
}


#endif
