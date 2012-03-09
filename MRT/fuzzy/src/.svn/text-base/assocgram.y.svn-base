%{
#include <getFuzzy.h> //fuzzy/include

#ifdef DMALLOC
#include <dmalloc.h>
#endif

#define AAERROR_VERBOSE 1

extern int assoclex();
int assocerror(char *);
extern int assocleng;

association_list *al=NULL;
association * a;
%}

%union {
  char * dataname;
}

%type <dataname> name 
%token <dataname> NAME
%token LEFTPAR
%token RIGHTPAR

%start associationlist

%%

associationlist: /*empty*/
               | association associationlist
               ;

association: LEFTPAR name name RIGHTPAR { a->set_label($2);
                                          a->set_shape($3);
                                          al->add_association(a);
					  free ($2);
					  free ($3);
					  a = new association();
                                        }
             ;

name: NAME {$$=(char *) malloc(strlen($1)+1);
             strcpy($$,$1);}
       ;

%%

int assocerror(char *s)
{
  printf("%s\n",s);
  return 0;
}

association_list * assoc_parser()
{
    al = new association_list();
    a = new association();
    assocparse();
    delete a;
    return al;
}
