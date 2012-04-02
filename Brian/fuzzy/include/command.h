/***************************************************************************
                          command.h  -  description
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


#ifndef command_h
#define command_h 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>

#ifdef DMALLOC

#include <dmalloc.h>

#endif

/**
 * Command to sent to actuators
 * @short Command for actuators
 * @see command_list
 * @see defuzzyfier
 */
class command 
{
  public:
      /**
       * Constructor
       * @return It returns an empty command
       */
      command();

      /**
       * Copy constructor.
       * @param right Object to be copied.
       * @returns It returns the copy of the given object.
       */
      command(const command &right);

      /**
       * Constructor.
       * @param alabel Command label
       * @param aset_point Command set point
       * @return It returns a command with given label and set point.
       */command (const char *alabel, float aset_point);

      /**
       * Destructor.
       */
      virtual ~command();

      /**
       * Method to retrieve command label
       * @return A pointer to command label.
       */
      const char * get_label ();
      /**
       * Method to set command label.
       * @param value Pointer to new command label. It must be freed after call.
       */
      void set_label (const char * value);

      //	Set point of the command
      /**
       * Method to retrieve command set point.
       * @return command set point.
       */
      float get_set_point ();
      /**
       * Method to set set-point.
       * @param value New set point.
       */
      void set_set_point (float value);

  private:
    // Data Members for Class Attributes
      char *label;
       float set_point;
};

// Class command

inline const char * command::get_label ()
{
  return label;
}

inline void command::set_label (const char * value)
{
  if (label!=NULL) free(label);
  label=(char *) malloc(strlen(value)+1);
  strcpy(label,value);
}

inline float command::get_set_point ()
{
  return set_point;
}

inline void command::set_set_point (float value)
{
  set_point = value;
}

#endif
