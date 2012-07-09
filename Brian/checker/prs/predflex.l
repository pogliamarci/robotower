%{
#include "predgram.tab.h"

  extern int linenum;
%}

%option noyywrap
%pointer

LETTERS   [a-zA-Z0-9_.]
BLANKS    [" "\t]

%%

"#".*		/* eat up comments */
"#("[^";"]*";"	/* eat up comments */
{BLANKS}+    /* eat up blank spaces */
";"          return NEWLINE;
"P"          return P;
"D"          return D;
"("          return LEFTPAR;
")"          return RIGHTPAR;
\n           linenum++;
"="          return DEF;
"NOT"        return NOT;
"AND"        return AND;
"OR"         return OR;
{LETTERS}+   {*(yytext+yyleng)='\0';
             predlval.dataname=yytext;
             return NAME;}


%%
