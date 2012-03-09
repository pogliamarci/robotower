%{
#include "rulesgram.tab.h"

extern int linenum;

%}

%option noyywrap
%pointer

LETTERS   [a-zA-Z0-9_"*"]
BLANK     [" "\t]

%%

"#".*		/* eat up comments */
"#("[^";"]*";"	/* eat up comments */
{BLANK}+	/* eat up blank spaces */
";"		return NEWLINE;
"("		return LEFTPAR;
")"		return RIGHTPAR;
\n		linenum++;
"=>"		return THEN;
"NOT"		return NOT;
"AND"		return AND;
"OR"		return OR;
"."		return SEPARATOR;
"&"		return START;
"DEL"		return MODALITY;
"SOMEBEHAVIOR"	return SOMEBEHAVIOR;
{LETTERS}+	{*(yytext+yyleng)='\0';
		ruleslval.dataname=yytext;
                return NAME;}
%%
