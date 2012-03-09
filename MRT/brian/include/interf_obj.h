/***************************************************************************
                          interf_obj.h  -  description
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

#ifndef interf_obj_h
#define interf_obj_h 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>

#ifdef DMALLOC

#include <dmalloc.h>

#endif


// stl
#include <stl.h>

#include <getFuzzy.h>

#include "predicate.h"
#include "predicate_list.h"
#include "predicate_list_debug.h"
#include "predicate_multimap.h"
#include "proposed_action.h"
#include "proposed_action_list_debug.h"
#include "proposed_action_list.h"
#include "proposed_action_multimap.h"
#include "weight_want.h"
#include "weight_want_list_debug.h"
#include "weight_want_list.h"
#include "weight_want_multimap.h"




#endif
