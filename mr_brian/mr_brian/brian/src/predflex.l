%{
#include "engine_objects.h"
#include "predgram.tab.h"

#ifdef DMALLOC
#include <dmalloc.h>
#endif

%}

%option noyywrap
%pointer

LETTERS   [a-zA-Z0-9_]
BLANK     [" "\t]

%%

"#".*		/* eat up comments */
"#("[^";"]*";"	/* eat up comments */
{BLANK}+     /* eat up blank spaces */
\n           /* eat up carriage rerturn */
"P"          return P;
"D"          return D;
"("          return LEFTPAR;
")"          return RIGHTPAR;
";"           return NEWLINE;
"="          return DEF;
"NOT"        return NOT;
"AND"        return AND;
"OR"         return OR;
{LETTERS}+   {*(yytext+yyleng)='\0';
             predlval.dataname=yytext;
             return NAME;}
%%
