/***************************************************************************
                          action.h  -  description
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

#ifndef action_h
#define action_h 1

#ifdef DMALLOC

#include <dmalloc.h>

#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>


//	Class that implements the actions composed after
//	behavior proposal.
/**
 * Action that exits from composer and goes to defuzzyfier
 * @short Action object
 * @see action_list
 * @see composer
 * @see defuzzyfier
 */
class action
{
  public:
      /**
       * Empty constructor
       * @return It returns an empty action.
       */
      action();

      /**
       * Copy constructor
       * @param right The object to be copied
       * @return It returns a copy of the given action
       */
      action(const action &right);

      /**
       * Constructor
       * @param aname Name of action
       * @param alabel Label of action
       * @return It returns an action with given name, label, and membership value set to 0.
       */
      action (const char *aname, const char *alabel);

      /**
       * Constructor
       * @param aname Name of action
       * @param alabel Label of action
       * @param am_f Membership value of action
       * @return It returns an action with given name, label and membership value
       */
      action (const char *aname, const char *alabel, float am_f);

      //SM: costruttore aname, alabel, am_f, arel (=reliability)
		action (const char *aname, const char *alabel, float am_f, float arel);
		
		
		/**
       * Destructor
       */
      virtual ~action();

      /**
       * Method to retrieve action name
       * @return A pointer to action name.
       */
      virtual const char * get_name ();

      /**
       * Method to set action name
       * @param value Pointer to new actio name. It must be freed after call.
       */
      void set_name (const char * value);

      /**
       * Method to retrieve action label
       * @return A pointer to action label.
       */
      const char * get_label ();

      /**
       * Method to set action label.
       * @param value Pointer to new action label. It must be freed after call.
       */
      void set_label (const char * value);

      //	Membership value of the action as the result of
      //	activation predicates.
      /**
       * Method to retrieve membership value.
       * @return Action membership value.
       */
      float get_membership_value ();

      /**
       * Method to set membership value.
       * @param value New membership value.
       */
      void set_membership_value (float value);

      //SM: Metodi get e set della reliability
		float get_reliability_value();
		void set_reliability_value(float areliability_value);
	   
  protected:
    char * name;

  private:
    // Data Members for Class Attributes
      char *label;
      float membership_value;
  		float reliability_value;
};


// Class action
inline const char * action::get_name ()
{
  return name;
}

inline void action::set_name (const char * value)
{
  if (name!=NULL) free(name);
  name=(char *) malloc(strlen(value)+1);
  strcpy(name,value);
}

inline const char * action::get_label ()
{
  return label;
}

inline void action::set_label (const char * value)
{
  if (label!=NULL) free(label);
  label=(char *) malloc(strlen(value)+1);
  strcpy(label,value);
}

inline float action::get_membership_value ()
{
  return membership_value;
}

inline void action::set_membership_value (float value)
{
  membership_value = value;
}

//SM: get e set per la reliability
inline float action:: get_reliability_value()
{
	return reliability_value;
}

inline void action::set_reliability_value(float areliability_value)
{
	reliability_value=areliability_value;
}
#endif
