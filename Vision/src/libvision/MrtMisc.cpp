#include "MrtMisc.h"

int getTime()
{
  struct timeval tv;
  struct timezone tz;
  gettimeofday(&tv,&tz);
  return(tv.tv_sec * 1000000 + tv.tv_usec);
}
