%{
#include "assgram.tab.h"

extern int linenum;
%}

%option noyywrap
%pointer

LETTERS   [a-zA-Z0-9_./]
DIG     [0-9]
BLANK     [" "\t]

%%
"#".*        /* eat up comments */
\n           linenum++;/* eat up all newline*/
{BLANK}+     /* eat up blank spaces */
"("          return LEFTPAR;
")"          return RIGHTPAR;
"level"      return LEVEL;
{DIG}+       return VALUE;
{LETTERS}+   { *(asstext+assleng)='\0';
               asslval.dataname=asstext;
               return NAME;}

%%
