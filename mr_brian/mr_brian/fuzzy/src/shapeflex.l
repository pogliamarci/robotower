%{
#include <getFuzzy.h>//fuzzy/include
#include "shapegram.tab.h"

#ifdef DMALLOC
#include <dmalloc.h>
#endif

//Added by mr:021128
void shape_cancel_memory_leaks()
{
  /* For non-reentrant C scanner only. */ 
  shape_delete_buffer(YY_CURRENT_BUFFER);
  yy_init = 1;
}

%}

%option noyywrap
%pointer

DIG       [0-9]
LETTERS   [a-zA-Z0-9_]
BLANK     [" "\t]

%%

"#".*        /* eat up comments */
\n           /* eat up all newline*/
{BLANK}+     /* eat up blank spaces */
"("          return LEFTPAR;
")"          return RIGHTPAR;
"TRI"        return TRI;
"TOL"        return TOL;
"TOR"        return TOR;
"DIV"        return DIV;
"TRA"        return TRA;
"REC"        return REC;
"SNG"        return SNG;
[-+]?{DIG}+(\.{DIG}+)? { *(shapetext+shapeleng)='\0';
                         shapelval.dataname=shapetext;
                         return VALUE;}
{LETTERS}+   { *(shapetext+shapeleng)='\0';
               shapelval.dataname=shapetext;
               return NAME;}
%%
