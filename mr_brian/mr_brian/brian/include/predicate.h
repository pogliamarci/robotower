/***************************************************************************
                          predicate.h  -  description
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

#ifndef predicate_h
#define predicate_h 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>

#ifdef DMALLOC

#include <dmalloc.h>

#endif




//	Class that relize the idea of a predicate or of a behavior.
/**
 * Parent class for predicates and active behaviors. Other classes are derived from it.
 * @short Parent class for predicates and active behaviors.
 * @see predicate_list
 */
class predicate 
{
  public:
      /**
       * Constructor
       * @return It returns an empty object
       */
      predicate();

      /**
       * Copy constructor.
       * @param right Object to be copied.
       * @returns It returns the copy of the given object.
       */
      predicate(const predicate &right);

      /**
       * Constructor.
       * @param aname Name of the object
       * @param avalue Value of the object
       * @return It returns an with given name and value, and reliability set to 1
       */
      predicate (const char *aname, float avalue);

      //## Operation: predicate%940324752
      //	Constructor: sets name to aname, value to avalue and
      //	reliability to arelibility.
      /**
       * Constructor
       * @param aname Name of the object
       * @param avalue Value of the object
       * @param areliability Reliability of the object
       * @return It returns an obejct with given name, value and reliability
       */
      predicate (const char *aname, float avalue, float areliability);

      /**
       * Destructor.
       */
      virtual ~predicate();

      //	Name of the predicate or of the behavior.
      /**
       * Method to retrieve object name.
       * @return A pointer to object name.
       */
      const char * get_name ();
      /**
       * Method to set object name.
       * @param value Pointer to new object name. It must be freed after call.
       */
      void set_name (const char * value);

      /**
       * Method to retrieve object value.
       * @return Object value.
       */
      float get_value ();
      /**
       * Method to set object value.
       * @param value New object value.
       */
      void set_value (float Value);

      /**
       * Method to retrieve object reliability.
       * @return Object reliability; 0 means not reliable.
       */
      float get_reliability ();
      /**
       * Method to set object reliability.
       * @param value New reliability value. 0 means not reliable.
       */
      void set_reliability (float value);

  private:
    // Data Members for Class Attributes

      char *name;
      float value;
      float reliability;
};

// Class predicate

inline const char * predicate::get_name ()
{
  return name;
}

inline void predicate::set_name (const char * value)
{
  if (name!=NULL) free(name);
  name=(char *) malloc(strlen(value)+1);
  strcpy(name,value);
}

inline float predicate::get_value ()
{
  return value;
}

inline void predicate::set_value (float Value)
{
  value = Value;
}

inline float predicate::get_reliability ()
{
  return reliability;
}

inline void predicate::set_reliability (float value)
{
  reliability = value;
}

#endif
