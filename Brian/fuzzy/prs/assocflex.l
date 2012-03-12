%{
#include <getFuzzy.h>
#include "assocgram.tab.h"


#ifdef DMALLOC
#include <dmalloc.h>
#endif

%}

%option noyywrap
%pointer

LETTERS   [a-zA-Z0-9_]
BLANK     [" "\t]

%%

"#".*        /* eat up comments */
\n           /* eat up all newline*/
{BLANK}+     /* eat up blank spaces */
"("          return LEFTPAR;
")"          return RIGHTPAR;
{LETTERS}+   { *(assoctext+assocleng)='\0';
               assoclval.dataname=assoctext;
               return NAME;}
%%
