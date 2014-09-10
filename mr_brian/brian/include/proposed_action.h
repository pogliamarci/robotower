/***************************************************************************
                          proposed_action.h  -  description
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

#ifndef proposed_action_h
#define proposed_action_h 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>

#ifdef DMALLOC

#include <dmalloc.h>

#endif
#include <getFuzzy.h>



//	Class used to represent the anctions proposed by
//	behavors.
/**
 * Action proposed by a behavior, to be composed with other ones.
 * @short Action poposed by a behavior
 * @see proposed_action_list
 * @see behavior
 * @see behavior_engine
 * @see composer
 */
class proposed_action : public action
{
  public:
      /**
       * Empty constructor
       * @return It returns an empty action.
       */
      proposed_action();

      /**
       * Copy constructor
       * @param right The object to be copied
       * @return It returns a copy of the given action
       */
      proposed_action(const proposed_action& right);

      /**
       * Constructor
       * @param aname Name of action
       * @param alabel Label of action
       * @param abehavior_name Name of the behavior that poposed this action
       * @return It returns an action with given name, label, behavior name and membership value set to 0.
       */
      proposed_action (const char *aname, const char *alabel, const char * abehavior_name);

      /**
       * Constructor
       * @param aname Name of action
       * @param alabel Label of action
       * @param abehavior_name Name of the behavior that poposed this action
       * @param am_f Membership value of action
       * @return It returns an action with given name, label, behavior name and membership value
       */
      proposed_action (const char *aname, const char *alabel, const char * abehavior_name, float am_f);

      /**
       * Destructor
       */
      virtual ~proposed_action();

      /**
       * Method to retrieve the name of the behavior that proposed the action
       * @return A pointer to the name.
       */
      const char * get_behavior_name ();


      /**
       * Method to set the name of the behavior that proposed the name
       * @param value Pointer to new name. It must be freed after call.
       */
      void set_behavior_name (const char * value);

      char * get_modality();

      const char * get_name();


  private:
    // Data Members for Class Attributes

      char *behavior_name;
};

// Class proposed_action

inline const char * proposed_action::get_behavior_name ()
{
  return behavior_name;
}

inline void proposed_action::set_behavior_name (const char * value)
{
  if (behavior_name!=NULL) free(behavior_name);
  behavior_name=(char *) malloc(strlen(value)+1);
  strcpy(behavior_name,value);
}

inline char * proposed_action::get_modality()
{
   char * tmp = (char *)malloc(4);
   char * x = (char *)name;
   if (*x != '&'){
     strcpy(tmp,"ADD");
      }
   else{
      tmp[0] = name[1];
      tmp[1] = name[2];
      tmp[2] = name[3];
      tmp[3] = '\0';
   }
   return tmp;
}

inline const char * proposed_action::get_name ()
{
  char * tmp_name = (char *)malloc(strlen(name)+1);
  char * x = (char *)name;
  int i;
  if (*x == '&') {
      for ( i = 5; name[i] != '\0'; i++) {
         tmp_name[i-5]=name[i];
         }
      tmp_name[i-5] = '\0';
      }
   else  strcpy(tmp_name, name);
   return((const char *)tmp_name);

}


#endif
