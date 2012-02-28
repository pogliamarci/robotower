%{
#include "interf_obj.h"
#include "rules_behav.h"
#include "preacher.h"
#include "rulesgram.tab.h"

#ifdef DMALLOC
#include <dmalloc.h>
#endif

extern unsigned int line_num;

%}

%option noyywrap
%pointer

LETTERS   [a-zA-Z0-9_&".""*"]
BLANK     [" "\t]

%%

"#".*		/* eat up comments */
"#("[^";"]*";"	/* eat up comments */
{BLANK}+	/* eat up blank spaces */
\n		line_num++;/* eat up return carriage */
"("		return LEFTPAR;
")"		return RIGHTPAR;
";"		return NEWLINE;
"=>"		return THEN;
"NOT"		return NOT;
"AND"		return AND;
"OR"		return OR;
{LETTERS}+	{*(rulestext+rulesleng)='\0';
		ruleslval.dataname=rulestext;
		return NAME;}
%%
