#ifndef crisp_data_multimap_h
#define crisp_data_multimap_h 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>

#ifdef DMALLOC

#include <dmalloc.h>

#endif


#include <stl.h>

#include "crisp_data.h"
#include "ltstr.h"

//	Abstract class that represents a multimap of crisp_data.
/**
 * Instantiation of multimap template in order to create a crisp data list
 * @see crisp_data_list
 */
typedef multimap<const char *, crisp_data *, ltstr > crisp_data_multimap;

#endif
