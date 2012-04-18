#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stl.h>
#include <cstring>

#define MAXSTR 150

extern void shapeerror(const char *s);
extern FILE * shapein;
extern void shapeparse();
extern void asserror(const char *s);
extern FILE * assin;
extern void assparse();
extern void prederror(const char *s);
extern FILE * predin;
extern void predparse();
extern void ruleserror(const char *s);
extern FILE * rulesin;
extern void rulesparse();


struct lstr
{
  bool operator()(char* s1, char* s2) const
  {
    return strcmp(s1, s2) < 0;
  }
};

// elenco di nomi
typedef set<char *,lstr> charset;
// elenco di nomi associati ad altri nomi
typedef map<char *,charset *,lstr> cocmmap; 

charset * tmpcharset=new charset(),
        * predicates=new charset(),
        * behavlist=new charset(),
        * behavfiles=new charset();

cocmmap * shapelist=new cocmmap(),
        * fuzzydata=new cocmmap(),
        * action=new cocmmap();
 
char * filename;
char * basepath;

enum checkstat {
  ctofass,
  ctofshape,
  pred,
  behav,
  cando,
  want,
  ftocshape,
  ftocass,
  rules
};

checkstat stat;

int linenum=0;

void addass(char * name1, char * name2)
{
  switch (stat)
    {
    case ctofass:
      {
	// name1: nome dato crisp
	// name2: nome shape
	cocmmap::iterator i=shapelist->find(name2);
	if (i!= shapelist->end())
	  // associo il dato crisp a tutte le label della shape
	  fuzzydata->insert(pair<char *, charset *> (name1,(*i).second));
	else
	  {
	    /* ERRORE */ // non trovata la shape!!!
	    char * msg = (char *) malloc(25 + strlen(name2));
	    strcpy(msg,"la shape '");
	    strcat(msg,name2);
	    strcat(msg,"' non esiste!");
	    asserror(msg);
	    free(msg);
	  }
      } break;
    case behav:
      {
	//name1: nome del comportamento
	//name2: nome del file di configurazione
	FILE * tmp=fopen(name2,"r");
	if (tmp==NULL)
	  {
	    /* ERRORE */ //il file non esiste
	    char * msg = (char *) malloc(25 + strlen(name2));
	    strcpy(msg,"il file '");
	    strcat(msg,name2);
	    strcat(msg,"' non esiste!");
	    asserror(msg);
	    free(msg);
	  }
	else
	  {
	    fclose(tmp);
	    behavlist->insert(name1);
	    behavfiles->insert(name2);
	  }
      } break;
    case ftocass:
      {
	// name1: nome azione
	// name2: nome shape
	cocmmap::iterator i=shapelist->find(name2);
	if (i!= shapelist->end())
	  // associo l'azione a tutte le label della shape
	  action->insert(pair<char *, charset *> (name1,(*i).second));
	else
	  {
	    /* ERRORE */ // non trovata la shape!!!
	    char * msg = (char *) malloc(25 + strlen(name2));
	    strcpy(msg,"la shape '");
	    strcat(msg,name2);
	    strcat(msg,"' non esiste!");
	    asserror(msg);
	    free(msg);
	  }
      } break;
    default:
      {
	char * msg=(char *) malloc(61+strlen(name1)+strlen(name2));
	strcpy(msg,"viene definita l'associazione '");
	strcat(msg,name1);
	strcat(msg,"-");
	strcat(msg,name2);
	strcat(msg,"' in un file sbagliato!");
	asserror(msg);
	free(msg);
      }
    }
}

void addshape(char * name)
{
  switch (stat)
    {
    case ctofshape:
      {
	shapelist->insert(pair<char *, charset *>(name,tmpcharset));
	tmpcharset=new charset();
      } break;
    case ftocshape:
      {
	shapelist->insert(pair<char *, charset *>(name,tmpcharset));
	tmpcharset=new charset();
      } break;
    default:
      {
	char * msg=(char *) malloc(51+strlen(name));
	strcpy(msg,"viene definita la shape '");
	strcat(msg,name);
	strcat(msg,"' in un file sbagliato!");
	shapeerror(msg);
	free(msg);
      }
    }
}

void addfuzzyset(char * name,char * lab)
{
  if ((stat==ftocshape)&&(strcmp(lab,"SNG")!=0))
    {
      /* ERRORE */ // nella defuzz. si usano solo singleton
      char * msg=(char *) malloc(51+strlen(name));
      strcpy(msg,"la shape '");
      strcat(msg,name);
      strcat(msg,"' deve essere un singleton!");
      shapeerror(msg);
      free(msg);
    }
  else
    tmpcharset->insert(name);
}

void data_exists(char * name1, char * name2)
{
  switch (stat)
    {
    case pred:
      {
	// name1: nome dato fuzzy
	// name2: label dato fuzzy
	cocmmap::iterator i=fuzzydata->find(name1);
	if (i!=fuzzydata->end())
	  {
	    if (strcmp("ANY",name2) != 0)
	      {
		charset::iterator q=(*i).second->find(name2);
		if (q==(*i).second->end())
		  {
		    /* ERRORE */ // non esiste nessuna label per il dato fuzzy
		    char * msg=(char *) malloc(51+strlen(name1)+strlen(name2));
		    strcpy(msg,"non esiste alcuna label '");
		    strcat(msg,name2);
		    strcat(msg,"' per il dato fuzzy '");
		    strcat(msg,name1);
		    strcat(msg,"'!");
		    prederror(msg);
		    free(msg);
		  }
	      }
	  }
	else
	  {
	    /* ERRORE */ //dato fuzzy non trovato!
	    char * msg=(char *) malloc(51+strlen(name1));
	    strcpy(msg,"non e' stato definito alcun dato fuzzy '");
	    strcat(msg,name1);
	    strcat(msg,"'!");
	    prederror(msg);
	    free(msg);
	  }
      } break;
    case rules:
      {
	// name1: nome azione
	// name2: label azione
	cocmmap::iterator i=action->find(name1);
	if (i!=action->end())
	  {
	    if (strcmp("ANY",name2) != 0 && name2[0] != '*')
	      {
		charset::iterator q=(*i).second->find(name2);
		if (q==(*i).second->end())
		  {
		    /* ERRORE */ // non esiste nessuna label per l'azione
		    char * msg=(char *) malloc(51+strlen(name1)+strlen(name2));
		    strcpy(msg,"non esiste alcuna label '");
		    strcat(msg,name2);
		    strcat(msg,"' per l'azione '");
		    strcat(msg,name1);
		    strcat(msg,"'!");
		    linenum--;
		    ruleserror(msg);
		    linenum++;
		    free(msg);
		  }
	      }
	  }
	else
	  {
	    /* ERRORE */ //azione non trovata!
	    char * msg=(char *) malloc(51+strlen(name1));
	    strcpy(msg,"non e' stata definita alcuna azione '");
	    strcat(msg,name1);
	    strcat(msg,"'!");
	    linenum--;
	    ruleserror(msg);
	    linenum++;
	    free(msg);
	  }
      } break;
    default:
      {
	/* ERRORE */ // i dati fuzzy valgono solo nella definizione dei predicati o le azioni solo per i file di regole
	char * msg=(char *) malloc(51+strlen(name1));
	strcpy(msg,"si fa riferimento al dato fuzzy od all'azione '");
	strcat(msg,name1);
	strcat(msg,"' in un file sbagliato!");
	prederror(msg);
	free(msg);
      }
    }
}

void pred_exists(char *name)
{
  charset::iterator i=predicates->find(name);
  if (i==predicates->end())
    {
      /* ERRORE */ // non esiste alcun predicato del genere!
      char * msg=(char *) malloc(30+strlen(name));
      strcpy(msg,"il predicato '");
      strcat(msg,name);
      strcat(msg,"' non esiste!");
      prederror(msg);
      free(msg);
    }
}


void beh_exists(char * name)
{
	//name: nome del comportamento
	charset::iterator i=behavlist->find(name);
	if (i==behavlist->end())
	  {
	    /* ERRORE */ //il comportamento non esiste
	    char * msg=(char *) malloc(30+strlen(name));
	    strcpy(msg,"il comportamento '");
	    strcat(msg,name);
	    strcat(msg,"' non esiste!");
	    prederror(msg);
	    free(msg);
	  }
      }


void addpred(char *name)
{
  switch (stat)
    {
    case pred:
      {
	//name: nome del predicato
	predicates->insert(name);
      } break;
    case cando:
      {
	//name: nome del comportamento
	charset::iterator i=behavlist->find(name);
	if (i==behavlist->end())
	  {
	    /* ERRORE */ //il comportamento non esiste
	    char * msg=(char *) malloc(30+strlen(name));
	    strcpy(msg,"il comportamento '");
	    strcat(msg,name);
	    strcat(msg,"' non esiste!");
	    prederror(msg);
	    free(msg);
	  }
      } break;
    case want:
      {
	//name: nome del comportamento
	charset::iterator i=behavlist->find(name);
	if (i==behavlist->end())
	  {
	    /* ERRORE */ //il comportamento non esiste
	    char * msg=(char *) malloc(30+strlen(name));
	    strcpy(msg,"il comportamento '");
	    strcat(msg,name);
	    strcat(msg,"' non esiste!");
	    prederror(msg);
	    free(msg);
	  }
      } break;
    default:
      {
	/* ERRORE */ // i predicati o nomi comportamenti valgono solo nella definizione dei predicati, nelle cando o nelle want
	char * msg=(char *) malloc(51+strlen(name));
	strcpy(msg,"si fa riferimento al predicato o al comportmento '");
	strcat(msg,name);
	strcat(msg,"' in un file sbagliato!");
	prederror(msg);
	free(msg);
      }
    }
}

int main(int argc, char ** argv)
{
  if (argc==2)
    {
      int l=strlen(argv[1]);
      basepath=(char *) malloc (l+2);
      strcpy(basepath,argv[1]);
      if (basepath[l-1]!='/')
	{
	  strcat(basepath,"/");
	}
    }
  else
    basepath="../config/";

  linenum=1;
  filename=(char *) malloc(MAXSTR);
  strcpy(filename,basepath);
  strcat(filename,"shape_ctof.txt");
  if ((shapein=fopen(filename,"r"))!=NULL)
    {
      stat=ctofshape;
      shapeparse();
      fclose(shapein);
    }
  else
    {
      // ERRORE   //file non trovato
      cout << filename << " non trovato!\n";
    }

  linenum=1;
  free(filename);
  filename=(char *) malloc(MAXSTR);
  strcpy(filename,basepath);
  strcat(filename,"ctof.txt");
  if ((assin=fopen(filename,"r"))!=NULL)
    {
      stat=ctofass;
      assparse();
      fclose(assin);
    }
  else
    {
      // ERRORE   //file non trovato
      cout << filename << " non trovato!\n";
    }

  shapelist=new cocmmap();

  linenum=1;
  free(filename);
  filename=(char *) malloc(MAXSTR);
  strcpy(filename,basepath);
  strcat(filename,"Predicate.ini");
  if ((predin=fopen(filename,"r"))!=NULL)
    {
      stat=pred;
      predparse();
      fclose(predin);
    }
  else
    {
      // ERRORE   //file non trovato
      cout << filename << " non trovato!\n";
    }

  linenum=1;
  free(filename);
  filename=(char *) malloc(MAXSTR);
  strcpy(filename,basepath);
  strcat(filename,"PredicateActions.ini");
  if ((predin=fopen(filename,"r"))!=NULL)
    {
      stat=pred;
      predparse();
      fclose(predin);
    }
  else
    {
      // ERRORE   //file non trovato
      cout << filename << " non trovato!\n";
    }

  linenum=1;
  free(filename);
  filename=(char *) malloc(MAXSTR);
  strcpy(filename,basepath);
  strcat(filename,"behaviour.txt");
  if ((assin=fopen(filename,"r"))!=NULL)
    {
      stat=behav;
      assparse();
      fclose(assin);
    }
  else
    {
      // ERRORE   //file non trovato
      cout << filename << " non trovato!\n";
    }
  
  linenum=1;
  free(filename);
  filename=(char *) malloc(MAXSTR);
  strcpy(filename,basepath);
  strcat(filename,"Cando.ini");
  if ((predin=fopen(filename,"r"))!=NULL)
    {
      stat=cando;
      predparse();
      fclose(predin);
    }
  else
    {
      // ERRORE   //file non trovato
      cout << filename << " non trovato!\n";
    }
  linenum=1;
  free(filename);
  filename=(char *) malloc(MAXSTR);
  strcpy(filename,basepath);
  strcat(filename,"Want.ini");
  if ((predin=fopen(filename,"r"))!=NULL)
    {
      stat=want;
      predparse();
      fclose(predin);
    }
  else
    {
      // ERRORE   //file non trovato
      cout << filename << " non trovato!\n";
    }
  
  linenum=1;
  free(filename);
  filename=(char *) malloc(MAXSTR);
  strcpy(filename,basepath);
  strcat(filename,"s_shape.txt");
  if ((shapein=fopen(filename,"r"))!=NULL)
    {
      stat=ftocshape;
      shapeparse();
      fclose(shapein);
    }
  else
    {
      // ERRORE   //file non trovato
      cout << filename << " non trovato!\n";
    }

  linenum=1;
  free(filename);
  filename=(char *) malloc(MAXSTR);
  strcpy(filename,basepath);
  strcat(filename,"s_ftoc.txt");
  if ((assin=fopen(filename,"r"))!=NULL)
    {
      stat=ftocass;
      assparse();
      fclose(assin);
    }
  else
    {
      // ERRORE   //file non trovato
      cout << filename << " non trovato!\n";
    }

  stat=rules;
  free(filename);
  charset::iterator i=behavfiles->begin();
  while (i!=behavfiles->end())
    {
      filename=*i;
      linenum=1;
      if ((rulesin=fopen(*i,"r"))!=NULL)
	{
	  rulesparse();
	  fclose(rulesin);
	}
      else
	cout << *i << " non trovato!\n";
      i++;
    }
}
