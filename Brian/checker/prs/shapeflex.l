%{
#include "shapegram.tab.h"

extern int linenum;
%}

%option noyywrap
%pointer

LETTERS   [a-zA-Z0-9_.]
DIG     [0-9]
BLANK     [" "\t]

%%

"#".*        /* eat up comments */
\n           linenum++;/* eat up all newline*/
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
[-+]?{DIG}+(\.{DIG}+)? return VALUE;
{LETTERS}+   { *(shapetext+shapeleng)='\0';
               shapelval.dataname=shapetext;
               return NAME;}

%%
