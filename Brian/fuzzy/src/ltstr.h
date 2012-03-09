#ifndef ltstr_h
#define ltstr_h 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>

#ifdef DMALLOC

#include <dmalloc.h>

#endif


/**
 * Struct ltstr is used by maps to compare two strings. It redefines operator ()
 * in order to return 0 if two strings are equal, -1 if s1 is lower than s2 and 1
 * if greater.
 */
struct ltstr {
  bool operator() (const char *s1, const char *s2) const
  {
    return strcmp(s1,s2)<0;
  }
};

#endif
