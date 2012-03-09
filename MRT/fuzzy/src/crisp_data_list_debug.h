/***************************************************************************
                          crisp_data_list_debug.h  -  description
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


//	This module declares crisp_data_list_debug.h
#ifndef crisp_data_list_debug_h
#define crisp_data_list_debug_h 1


#ifdef DMALLOC

#include <dmalloc.h>

#endif


#include "crisp_data_list.h"

/**
 * Function to output crisp_data_list content. For debug purposes only.
 * @see crisp_data_list
 */
void viewcdl(crisp_data_list * cdl);

#endif
