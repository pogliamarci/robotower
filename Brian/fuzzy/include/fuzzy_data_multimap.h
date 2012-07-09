#ifndef fuzzy_data_multimap_h
#define fuzzy_data_multimap_h 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>


#include "fuzzy_data.h"
#include "ltstr.h"


#ifdef DMALLOC

#include <dmalloc.h>

#endif


// stl
#include <stl.h>


/**
 * Instantiation of multimap template in order to create a fuzzy data list
 * @see fuzzy_data_list
 */
typedef multimap<const char *, fuzzy_data *, ltstr > fuzzy_data_multimap;

#endif
