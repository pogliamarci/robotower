#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <gtk/gtk.h>

#include "callbacks.h"
#include "interface.h"
#include "support.h"

#define GCC_VER 33
#include "interf_obj.h"
#include "brian.h"

#define GLADE_HOOKUP_OBJECT(component,widget,name) \
  g_object_set_data_full (G_OBJECT (component), name, \
    gtk_widget_ref (widget), (GDestroyNotify) gtk_widget_unref)

#define GLADE_HOOKUP_OBJECT_NO_REF(component,widget,name) \
  g_object_set_data (G_OBJECT (component), name, widget)


//VARIABILI GLOBALE (FINESTRE PER LE LISTE E BRIAN)
MrBrian *brian_the_brain=NULL;
GetFuzzy *fuz=NULL;
crisp_data_list *cdl=NULL;
const gchar *project=NULL;
guint it=0; //iterazioni di MrBrian

GtkWidget *crispList=NULL;//per la finestra che mostra i dati crisp
GtkWidget *fuzzyList=NULL;//per la finestra che mostra la fuzzy data list
GtkWidget *wwantList=NULL;//weight want list
GtkWidget *cmdList=NULL; //command list
GtkWidget *predList=NULL;//predicate list
GtkWidget *palList=NULL; //proposed action list
GtkWidget *actList=NULL; //action List
GtkWidget *windowino=NULL;
GtkWidget *mainWin=NULL;
//FINE VARIABILI


void on_nuovo1_activate(GtkMenuItem *menuitem, gpointer user_data)
{
GtkWidget *entry;
GtkWidget *window;
GtkWidget *status;
char string[7];
	
window=lookup_widget(GTK_WIDGET(menuitem),"mainWindow");
status=lookup_widget(GTK_WIDGET(menuitem),"statusbar1");
gtk_statusbar_push((GtkStatusbar*)status,0,"No Project File Name Selected");//modifico la status bar

it=0;//azzero il numero di iterazioni di MrBrian

	//ripulisco tutte le caselle di testo...
for (int i=1; i<=9; i++)
	{
	sprintf(string,"entry%d",i);
	g_print("\n%s",string);	
    entry=lookup_widget(GTK_WIDGET(menuitem),string);
	gtk_entry_set_text((GtkEntry*)entry,"");
	}

//dealloco tutti i puntatori di brian e/o liste esisenti...
if (brian_the_brain) { free((void*)brian_the_brain); brian_the_brain=NULL; } //BRIAN THE BRAIN
if (crispList) { gtk_widget_destroy(crispList); crispList=NULL; }
if (fuzzyList) { gtk_widget_destroy(fuzzyList); fuzzyList=NULL; }
if (wwantList) { gtk_widget_destroy(wwantList); wwantList=NULL; }
if (cmdList)   { gtk_widget_destroy(cmdList); cmdList=NULL; }
if (predList)  { gtk_widget_destroy(predList); predList=NULL; }
if (palList)  { gtk_widget_destroy(palList); palList=NULL; }
if (actList)  { gtk_widget_destroy(actList); actList=NULL; }

if (project)   { free((void*)project); project=NULL; } //il file del progetto

//disattivo tutti i menu e i pulsanti inutili
GtkWidget *item1=lookup_widget((GtkWidget*)menuitem,"crisp_data_list1");
GtkWidget *item2=lookup_widget((GtkWidget*)menuitem,"fuzzy_data_list1");
GtkWidget *item3=lookup_widget((GtkWidget*)menuitem,"weight_want_list1");
GtkWidget *item4=lookup_widget((GtkWidget*)menuitem,"command_list1");
GtkWidget *item5=lookup_widget((GtkWidget*)menuitem,"predicate_list1");
GtkWidget *item6=lookup_widget((GtkWidget*)menuitem,"addCrisp");
GtkWidget *item7=lookup_widget((GtkWidget*)menuitem,"runBrian");
GtkWidget *item8=lookup_widget((GtkWidget*)menuitem,"proposed_action_list1");
GtkWidget *item9=lookup_widget((GtkWidget*)menuitem,"action_list1");
gtk_check_menu_item_set_active((GtkCheckMenuItem*)item1,FALSE);
gtk_check_menu_item_set_active((GtkCheckMenuItem*)item2,FALSE);
gtk_check_menu_item_set_active((GtkCheckMenuItem*)item3,FALSE);
gtk_check_menu_item_set_active((GtkCheckMenuItem*)item4,FALSE);
gtk_check_menu_item_set_active((GtkCheckMenuItem*)item5,FALSE);
gtk_check_menu_item_set_active((GtkCheckMenuItem*)item8,FALSE);
gtk_check_menu_item_set_active((GtkCheckMenuItem*)item9,FALSE);
gtk_widget_set_sensitive(item1,FALSE);
gtk_widget_set_sensitive(item2,FALSE);
gtk_widget_set_sensitive(item3,FALSE);
gtk_widget_set_sensitive(item4,FALSE);
gtk_widget_set_sensitive(item5,FALSE);
gtk_widget_set_sensitive(item6,FALSE);
gtk_widget_set_sensitive(item7,FALSE);
gtk_widget_set_sensitive(item8,FALSE);
gtk_widget_set_sensitive(item9,FALSE);
}


void on_apri1_activate(GtkMenuItem *menuitem, gpointer user_data)
{
GtkWidget *entry;
GtkWidget *status;
GtkWidget *window;
char string[7];
const gchar *buf=NULL;
FILE *input=NULL;
                                                                                
project=get_filename((GtkWidget*)menuitem,user_data,"Apri Progetto");//acquisisco il nome del file di progetto da aprire
if (project) //se esiste...
	{
	input=fopen(project,"r");//tento di aprirlo...
	if (input)//...se riesco ad aprirlo
		{//leggo da file le locazioni dei file di configurazione di brian
		buf=(const gchar*)g_malloc(80);
		for (int i=1; i<=9; i++)
			{
			sprintf(string,"entry%d",i);
			g_print("\n%s",string);	
			entry=lookup_widget(GTK_WIDGET(menuitem),string);
			fscanf(input,"[ %s ]\n",buf);
			if (strncmp(buf,"empty",5)==0) gtk_entry_set_text((GtkEntry*)entry,"");
			else gtk_entry_set_text((GtkEntry*)entry,buf);
			}
		g_free((void*)buf);
		status=lookup_widget(GTK_WIDGET(menuitem),"statusbar1");
		window=lookup_widget(GTK_WIDGET(menuitem),"mainWindow");
		gtk_window_set_title(GTK_WINDOW(window),"Mr.Brian Testing Enviroment");
		gtk_statusbar_pop((GtkStatusbar*)status,0);
		gtk_statusbar_push((GtkStatusbar*)status,0,project);//metto il nome del file aperto nella status bar
		}
	else g_printerr("errore apertura file %s",project);
	fclose(input);
	}	
else g_printerr("errore lettura nome file");
}

void on_salva1_activate(GtkMenuItem *menuitem, gpointer user_data)
{
GtkWidget *entry;
GtkWindow *window;
GtkWidget *status;
char string[7];
const gchar *buf=NULL;
FILE *output=NULL;
	
//se non si sta già lavorando su un progetto, si richiede un nome per il nuovo progetto (come in Save as)
if (!project) project=get_filename((GtkWidget*)menuitem,user_data,"Nome progetto");
output=fopen(project,"w+");//tento di aprire il file in scrittura
if (output)//...se lo apre...
	{
	for (int i=1; i<=9; i++)
		{
		sprintf(string,"entry%d",i);
		g_print("\n%s",string);
		entry=lookup_widget(GTK_WIDGET(menuitem),string);
		buf=gtk_entry_get_text((GtkEntry*)entry); //leggo il contenuto della casella di testo...
		if (buf)
			{
			fprintf(output,"[ %s ]\n",buf); //scrivo il contenuto nel file di progetto
			}
		else g_printerr("errore lettura percorso dalla casella di testo");
		}
	fclose(output);
	status=lookup_widget(GTK_WIDGET(menuitem),"statusbar1");
	gtk_statusbar_pop((GtkStatusbar*)status,0);
	gtk_statusbar_push((GtkStatusbar*)status,0,project);//scrivo il nome del file nella status bar
	}
else g_printerr("errore creazione/apertura file destinazione");
}


void on_salva_come1_activate(GtkMenuItem  *menuitem, gpointer user_data)
{
GtkWidget *entry;
GtkWindow *window;
GtkWidget *status;
char string[7];
const gchar *buf=NULL;
FILE *output=NULL;
	
project=get_filename((GtkWidget*)menuitem,user_data,"Nome progetto");//acquisisco il nuovo nome del progetto
output=fopen(project,"w+");//tento di aprire il file in scrittura
if (output)//...se lo apre...
	{
	for (int i=1; i<=9; i++)
		{
		sprintf(string,"entry%d",i);
		g_print("\n%s",string);
		entry=lookup_widget(GTK_WIDGET(menuitem),string);
		buf=gtk_entry_get_text((GtkEntry*)entry); //leggo il contenuto della casella di testo...
		if (buf)
			{
			fprintf(output,"[ %s ]\n",buf); //scrivo il contenuto nel file di progetto
			}
		else g_printerr("errore lettura percorso dalla casella di testo");
		}
	fclose(output);
	status=lookup_widget(GTK_WIDGET(menuitem),"statusbar1");
	gtk_statusbar_pop((GtkStatusbar*)status,0);
	gtk_statusbar_push((GtkStatusbar*)status,0,project);//scrivo il nome del file nella status bar
	}
else g_printerr("errore creazione/apertura file destinazione");
}


void on_esci1_activate (GtkMenuItem *menuitem, gpointer user_data)
{
  gint x,y;
  gint sx,sy;
  
  if (crispList&&fuzzyList&&wwantList&&cmdList&&predList&&palList&&actList)
  	{
  	gtk_window_get_position((GtkWindow*)crispList,&x,&y);
  	gtk_window_get_size((GtkWindow*)crispList,&sx,&sy);
  	g_print("\nCrisp List: %d %d - %d %d",x,y,sx,sy);
  	gtk_window_get_position((GtkWindow*)fuzzyList,&x,&y);
  	gtk_window_get_size((GtkWindow*)fuzzyList,&sx,&sy);
  	g_print("\nFuzzy List: %d %d - %d %d",x,y,sx,sy);
  	gtk_window_get_position((GtkWindow*)wwantList,&x,&y);
  	gtk_window_get_size((GtkWindow*)wwantList,&sx,&sy);
  	g_print("\nWeight Want List: %d %d - %d %d",x,y,sx,sy);
	gtk_window_get_position((GtkWindow*)cmdList,&x,&y);
  	gtk_window_get_size((GtkWindow*)cmdList,&sx,&sy);
  	g_print("\nCommand List: %d %d - %d %d",x,y,sx,sy);
  	gtk_window_get_position((GtkWindow*)predList,&x,&y);
  	gtk_window_get_size((GtkWindow*)predList,&sx,&sy);
  	g_print("\nPredicate List: %d %d - %d %d",x,y,sx,sy);
  	gtk_window_get_position((GtkWindow*)palList,&x,&y);
  	gtk_window_get_size((GtkWindow*)palList,&sx,&sy);
  	g_print("\nPal List: %d %d - %d %d",x,y,sx,sy);
  	gtk_window_get_position((GtkWindow*)actList,&x,&y);
  	gtk_window_get_size((GtkWindow*)actList,&sx,&sy);
  	g_print("\nAction List: %d %d - %d %d",x,y,sx,sy);
	}
else g_print("\nLe finestre non ci sono!");
gtk_main_quit();
}

void on_about1_activate(GtkMenuItem *menuitem, gpointer user_data)
{

}


void on_browse1_clicked(GtkButton *button, gpointer user_data)
{
const gchar *filename=NULL;
GtkWidget *entry=lookup_widget(GTK_WIDGET(button),"entry1");
filename=get_filename((GtkWidget*)button,user_data,"Select Fuzzy Associations file");
if (filename) gtk_entry_set_text((GtkEntry*)entry,filename);
if (filename) g_free((void*)filename);
}

void on_browse2_clicked(GtkButton *button, gpointer user_data)
{
const gchar *filename=NULL;
GtkWidget *entry=lookup_widget(GTK_WIDGET(button),"entry2");
filename=get_filename((GtkWidget*)button,user_data,"Select Fuzzy Shapes file");
if (filename) gtk_entry_set_text((GtkEntry*)entry,filename);
if (filename) g_free((void*)filename);
}

void on_browse3_clicked(GtkButton *button,gpointer user_data)
{
const gchar *filename=NULL;
GtkWidget *entry=lookup_widget(GTK_WIDGET(button),"entry3");
filename=get_filename((GtkWidget*)button,user_data,"Select Defuzzy Associations file");
if (filename) gtk_entry_set_text((GtkEntry*)entry,filename);
if (filename) g_free((void*)filename);
}




void on_browse4_clicked(GtkButton *button, gpointer user_data)
{
const gchar *filename=NULL;
GtkWidget *entry=lookup_widget(GTK_WIDGET(button),"entry4");
filename=get_filename((GtkWidget*)button,user_data,"Select Defuzzy Shapes file");
if (filename) gtk_entry_set_text((GtkEntry*)entry,filename);
if (filename) g_free((void*)filename);
}



void on_browse5_clicked(GtkButton *button,gpointer  user_data)
{
const gchar *filename=NULL;
GtkWidget *entry=lookup_widget(GTK_WIDGET(button),"entry5");
filename=get_filename((GtkWidget*)button,user_data,"Select Behaviors files");
if (filename) gtk_entry_set_text((GtkEntry*)entry,filename);
if (filename) g_free((void*)filename);
}


void on_browse6_clicked(GtkButton *button, gpointer user_data)
{
const gchar *filename=NULL;
GtkWidget *entry=lookup_widget(GTK_WIDGET(button),"entry6");
filename=get_filename((GtkWidget*)button,user_data,"Select Candoes file");
if (filename) gtk_entry_set_text((GtkEntry*)entry,filename);
if (filename) g_free((void*)filename);
}


void on_browse7_clicked(GtkButton *button, gpointer user_data)
{
const gchar *filename=NULL;
GtkWidget *entry=lookup_widget(GTK_WIDGET(button),"entry7");
filename=get_filename((GtkWidget*)button,user_data,"Select Wants file");
if (filename) gtk_entry_set_text((GtkEntry*)entry,filename);
if (filename) g_free((void*)filename);
}


void on_browse8_clicked(GtkButton *button, gpointer user_data)
{
const gchar *filename=NULL;
GtkWidget *entry=lookup_widget(GTK_WIDGET(button),"entry8");
filename=get_filename((GtkWidget*)button,user_data,"Select Pries file");
if (filename) gtk_entry_set_text((GtkEntry*)entry,filename);
if (filename) g_free((void*)filename);
}


void on_browse9_clicked(GtkButton *button, gpointer user_data)
{
const gchar *filename=NULL;
GtkWidget *entry=lookup_widget(GTK_WIDGET(button),"entry9");
filename=get_filename((GtkWidget*)button,user_data,"Select Actionpries file");
if (filename) gtk_entry_set_text((GtkEntry*)entry,filename);
if (filename) g_free((void*)filename);
}


void on_addCrisp_clicked(GtkButton *button, gpointer user_data)
{
GtkWidget *title;
GtkWidget *table;
GtkWidget *name;
GtkWidget *value;
GtkWidget *reliability;
GtkWidget *entryName;
GtkWidget *entryValue;
GtkWidget *entryReliability;
GtkWidget *dataName;
GtkWidget *dataValue;
GtkWidget *dataReliability;
GtkWidget *window;
GtkWidget *crispDialog;
GtkWidget *box;
GtkWidget *delButton;
char val[6];


if (brian_the_brain)//se esiste brian aggiungo crisp...
	{
	if (!crispList) createCrispDataList((GtkWidget*)button,user_data);
	box=lookup_widget(GTK_WIDGET(button),"crispbox");
	window=lookup_widget(GTK_WIDGET(button),"mainWindow");
	crispDialog=gtk_dialog_new_with_buttons("Insert Crisp Data",(GtkWindow*)window,GTK_DIALOG_DESTROY_WITH_PARENT,
																	GTK_STOCK_OK,GTK_RESPONSE_OK,
																	GTK_STOCK_CANCEL,GTK_RESPONSE_CANCEL,NULL);
	title=gtk_label_new("Insert Crisp Data");
	gtk_box_pack_start(GTK_BOX (((GtkDialog*)(crispDialog))->vbox), title, FALSE, FALSE, 0);
	table=gtk_table_new (2,2,FALSE);
	gtk_box_pack_start(GTK_BOX(((GtkDialog*)(crispDialog))->vbox),table,FALSE,FALSE,0);
	
	//etichette
	dataName=gtk_label_new("Data Name:");
	gtk_widget_show(dataName);
	gtk_table_attach (GTK_TABLE (table),dataName,0,1,0,1,(GtkAttachOptions)(GTK_FILL),(GtkAttachOptions)(0),0,0);
	gtk_misc_set_alignment (GTK_MISC (dataName), 0, 0.5);
	dataValue=gtk_label_new("Data Value:");
	gtk_widget_show(dataValue);
	gtk_table_attach (GTK_TABLE (table),dataValue,0,1,1,2,(GtkAttachOptions)(GTK_FILL),(GtkAttachOptions)(0),0,0);
	gtk_misc_set_alignment (GTK_MISC (dataValue), 0, 0.5);
	dataReliability=gtk_label_new("Data Reliability:");
	gtk_widget_show(dataReliability);
	gtk_table_attach(GTK_TABLE (table),dataReliability,0,1,2,3,(GtkAttachOptions)(GTK_FILL),(GtkAttachOptions)(0),0,0);
	gtk_misc_set_alignment (GTK_MISC (dataReliability), 0, 0.5);
	
	//casella di testo e spinbutton
	entryName=gtk_entry_new();
	gtk_widget_show(entryName);
	gtk_table_attach (GTK_TABLE (table),entryName,1,2,0,1,(GtkAttachOptions)(GTK_FILL),(GtkAttachOptions)(0),0,0);
	entryValue=gtk_spin_button_new_with_range(0,10000,0.01);
	gtk_widget_show(entryValue);
	gtk_table_attach (GTK_TABLE (table),entryValue,1,2,1,2,(GtkAttachOptions)(GTK_FILL),(GtkAttachOptions)(0),0,0);
	entryReliability=gtk_spin_button_new_with_range(0,1,0.01);
	gtk_widget_show(entryReliability);
	gtk_table_attach (GTK_TABLE (table),entryReliability,1,2,2,3,(GtkAttachOptions)(GTK_FILL),(GtkAttachOptions)(0),0,0);
	gtk_widget_show(table);
	gtk_widget_show(title);
	if (gtk_dialog_run((GtkDialog*)crispDialog)==GTK_RESPONSE_OK)
		{//devo leggere il dato dal dialog e aggiungere alla crisp list
		g_print("\nName: %s Value: %f Reliability: %f",gtk_entry_get_text((GtkEntry*)entryName),
									  			                 gtk_spin_button_get_value((GtkSpinButton*)entryValue),
								                             gtk_spin_button_get_value((GtkSpinButton*)entryReliability));
		crisp_data *cd=new crisp_data(gtk_entry_get_text((GtkEntry*)entryName),
									  		   gtk_spin_button_get_value((GtkSpinButton*)entryValue),
								            gtk_spin_button_get_value((GtkSpinButton*)entryReliability));
		if (cd)
			if (cdl)
				(*cdl).add(cd);
			else g_printerr("errore, la crisp data list non esiste");
		else g_printerr("errore nella creazione del dato crisp da aggiungere");
	    GtkWidget *tab=gtk_table_new(1,4,FALSE);
		gtk_box_pack_start(GTK_BOX(box),tab,FALSE,FALSE,0); //true true
		name=gtk_entry_new();
		gtk_entry_set_text((GtkEntry*)name,(*cd).get_name());
		sprintf(val,"%f",(*cd).get_value());
		value=gtk_entry_new();
		gtk_entry_set_text((GtkEntry*)value,val);
		sprintf(val,"%f",(*cd).get_reliability());
		reliability=gtk_entry_new();
		gtk_entry_set_text((GtkEntry*)reliability,val);
		delButton=gtk_button_new_with_label("Del");
		gtk_table_attach (GTK_TABLE (tab),name,0,1,0,1,(GtkAttachOptions)(GTK_FILL),(GtkAttachOptions)(0),0,0);
		gtk_table_attach (GTK_TABLE (tab),value,1,2,0,1,(GtkAttachOptions)(GTK_FILL),(GtkAttachOptions)(0),0,0);
		gtk_table_attach (GTK_TABLE (tab),reliability,2,3,0,1,(GtkAttachOptions)(GTK_FILL),(GtkAttachOptions)(0),0,0);
    	gtk_table_attach (GTK_TABLE (tab),delButton,3,4,0,1,(GtkAttachOptions)(GTK_FILL),(GtkAttachOptions)(0),0,0);
		gtk_entry_set_editable((GtkEntry*)name,FALSE);
		gtk_entry_set_editable((GtkEntry*)value,FALSE);
		gtk_entry_set_editable((GtkEntry*)reliability,FALSE);
		gtk_widget_show(name);
		gtk_widget_show(value);
		gtk_widget_show(reliability);
		gtk_widget_show(delButton);
		gtk_widget_show(tab);
   	    g_signal_connect( (gpointer) delButton,"clicked",G_CALLBACK(on_del_crisp_clicked),NULL);//al click su delete il crisp data viene eliminato
        gtk_widget_destroy(crispDialog);
		}
	else gtk_widget_destroy(crispDialog);
	}
	else {//...altrimenti non ha senso
	 	displayMessage((GtkWidget*)button,user_data,"Sorry there's no brian.\n Can't add Crisp data");
	 	}	
}


void on_createBrian_clicked(GtkButton *button, gpointer user_data)
{
guint ret=createBrian((GtkWidget*)button,user_data);
const gchar *message;
if (ret==0) 
	{
	message="BRIAN OK! (returned 0)";
	GtkWidget *cl=lookup_widget((GtkWidget*)button,"crisp_data_list1");
    GtkWidget *ac=lookup_widget((GtkWidget*)button,"addCrisp");
	GtkWidget *run=lookup_widget((GtkWidget*)button,"runBrian");
	GtkWidget *reset=lookup_widget((GtkWidget*)button,"resetBrian");
	GtkWidget *hide_all=lookup_widget((GtkWidget*)button,"hide_all");
	GtkWidget *show_all=lookup_widget((GtkWidget*)button,"show_all");
	gtk_widget_set_sensitive(hide_all,TRUE);
	gtk_widget_set_sensitive(show_all,TRUE);
	gtk_widget_set_sensitive(cl,TRUE);
	gtk_widget_set_sensitive(ac,TRUE);
	gtk_widget_set_sensitive(run,TRUE);
	gtk_widget_set_sensitive(reset,TRUE);
	}
else if (ret==1) message="Alrealdy Exists (returned 1)";
else if (ret==-1) message="Create Mr.Brian: new MrBrian() Error (returned -1)";
else message="Cosa sclerata";
displayMessage((GtkWidget*)button,user_data,message);
}

void on_runBrian_clicked(GtkButton *button, gpointer user_data)
{
GtkWidget *item1,*item2,*item3,*item4,*item5,*item6,*item7;
weight_want_list::iterator w;
fuzzy_data_list::iterator f;
command_list::iterator c;
predicate_list::iterator p;
proposed_action_list::iterator pa;	
action_list::iterator a;

weight_want_list *wwl=NULL;
fuzzy_data_list *fdl=NULL;
command_list *cdl=NULL;
predicate_list *pdl=NULL;
proposed_action_list *pal=NULL;
action_list *al=NULL;
GtkWidget *window;

if (brian_the_brain)
	{
	it++;
	(*brian_the_brain).run();
	(*brian_the_brain).debug();
	window=lookup_widget((GtkWidget*)button,"mainWindow");
	mainWin=window;
	gtk_widget_hide((GtkWidget*)window);
	
	if (!crispList) createCrispDataList((GtkWidget*)button,user_data);
	if (!fuzzyList) createFuzzyDataList((GtkWidget*)button,user_data);
	if (!wwantList) createWeightWantList((GtkWidget*)button,user_data);
	if (!cmdList)   createCommandList((GtkWidget*)button,user_data);
	if (!predList)  createPredicateList((GtkWidget*)button,user_data);
	if (!palList)	createPalList((GtkWidget*)button,user_data);
	if (!actList)   createActList((GtkWidget*)button,user_data);
	item1=lookup_widget((GtkWidget*)button,"crisp_data_list1");
	item2=lookup_widget((GtkWidget*)button,"fuzzy_data_list1");
	item3=lookup_widget((GtkWidget*)button,"weight_want_list1");
	item4=lookup_widget((GtkWidget*)button,"command_list1");
	item5=lookup_widget((GtkWidget*)button,"predicate_list1");
	item6=lookup_widget((GtkWidget*)button,"proposed_action_list1");
	item7=lookup_widget((GtkWidget*)button,"action_list1");
	gtk_check_menu_item_set_active((GtkCheckMenuItem*)item1,TRUE);
	gtk_check_menu_item_set_active((GtkCheckMenuItem*)item2,TRUE);
	gtk_check_menu_item_set_active((GtkCheckMenuItem*)item3,TRUE);
	gtk_check_menu_item_set_active((GtkCheckMenuItem*)item4,TRUE);
	gtk_check_menu_item_set_active((GtkCheckMenuItem*)item5,TRUE);
	gtk_check_menu_item_set_active((GtkCheckMenuItem*)item6,TRUE);
	gtk_check_menu_item_set_active((GtkCheckMenuItem*)item7,TRUE);
	gtk_widget_set_sensitive(item2,TRUE);
	gtk_widget_set_sensitive(item3,TRUE);
	gtk_widget_set_sensitive(item4,TRUE);
	gtk_widget_set_sensitive(item5,TRUE);
	gtk_widget_set_sensitive(item6,TRUE);
	gtk_widget_set_sensitive(item7,TRUE);

	//Riempio le liste (non la crisp list)
	// 1) Riempio la weight want list
	wwl=(*brian_the_brain).get_weight_want_list();
	if (wwl)
		{
  	    char string[20];
		GtkWidget *wwantView=lookup_widget((GtkWidget*)button,"wwview");
		sprintf(string,"Mr. Brian->run(%d)",it);
		GtkListStore *store;
		store=(GtkListStore*)gtk_tree_view_get_model((GtkTreeView*)wwantView);
		GtkTreeIter iter;
		gtk_list_store_append(store,&iter);
		gtk_list_store_set(store,&iter,0,string,1,"",2,"",-1);
		for (w=wwl->begin(); w!=wwl->end();w++)
			{
			char value[10];
			sprintf(value,"%f",(*w).second->get_value());
			GtkTreeIter iter;
			gtk_list_store_append(store,&iter);
			gtk_list_store_set(store,&iter,0,(*w).second->get_name(),
										   1,value,-1);
			}
		}
	else displayMessage((GtkWidget*)button,user_data,"Something wrong, there's no Weight Want List");
	
	// 2) Riempio la fuzzy data list
	fdl=fuz->get_fuzzy_data_list();
	if (fdl)
		{
		char string[20];
		GtkWidget *fuzzyView=lookup_widget((GtkWidget*)button,"fuzzyview");
		sprintf(string,"Mr. Brian->run(%d)",it);
		GtkListStore *store;
		store=(GtkListStore*)gtk_tree_view_get_model((GtkTreeView*)fuzzyView);
		GtkTreeIter iter;
		gtk_list_store_append(store,&iter);
		gtk_list_store_set(store,&iter,0,string,1,"",2,"",-1);
		for (f=fdl->begin(); f!=fdl->end(); f++)
			{
			char membership[10];
			sprintf(membership,"%f",(*f).second->get_membership_value());
			GtkTreeIter iter;
			gtk_list_store_append(store,&iter);
			gtk_list_store_set(store,&iter,0,(*f).second->get_name(),
										   1,(*f).second->get_label(),
										   2,membership,-1);
			}
		}
	else displayMessage((GtkWidget*)button,user_data,"Something wrong, there's no Fuzzy Data List");
	
	// 3) Riempio la Command List
	cdl=fuz->get_command_singleton_list();
	if (cdl)
		{
		char string[20];
		GtkWidget *cmdView=lookup_widget((GtkWidget*)button,"cmview");
		sprintf(string,"Mr. Brian->run(%d)",it);
		GtkListStore *store;
        store=(GtkListStore*)gtk_tree_view_get_model((GtkTreeView*)cmdView);	
		GtkTreeIter iter;
		gtk_list_store_append(store,&iter);
		gtk_list_store_set (store,&iter,0,string,1,"",2,"",-1);
		for (c=cdl->begin();c!=cdl->end();c++)
			{
			char setpoint[10];
			sprintf(setpoint,"%f",(*c).second->get_set_point());
			GtkTreeIter iter;
			gtk_list_store_append(store,&iter);
			gtk_list_store_set (store,&iter,0,(*c).first,
											1,setpoint,-1);						
			}
		}
	else displayMessage((GtkWidget*)button,user_data,"Something wrong, there's no Command List (Singleton)");

	// 4) Riempio la Predicate List
	pdl=brian_the_brain->get_predicate_list();
	if (pdl)
		{
		char string[20];
		GtkWidget *predView=lookup_widget((GtkWidget*)button,"pview");
		sprintf(string,"Mr. Brian->run(%d)",it);
		GtkListStore *store;
        store=(GtkListStore*)gtk_tree_view_get_model((GtkTreeView*)predView);	
		GtkTreeIter iter;
		gtk_list_store_append(store,&iter);
		gtk_list_store_set (store,&iter,0,string,1,"",2,"",-1);
		for (p=pdl->begin();p!=pdl->end();p++)
			{
			char mv[10],rel[10];
			sprintf(mv,"%f",(*p).second->get_value());
			sprintf(rel,"%f",(*p).second->get_reliability());
			GtkTreeIter iter;
			gtk_list_store_append(store,&iter);
			gtk_list_store_set (store,&iter,0,(*p).second->get_name(),
											1,mv,
											2,rel,-1);			
			}
		}
	
	// 5) riempio la Proposed Action List
	pal=brian_the_brain->get_proposed_action_list();
	if (pal)
		{
		char string[20];
		sprintf(string,"Mr. Brian->run(%d)",it);
		GtkWidget *palView=lookup_widget((GtkWidget*)button,"palview");
		GtkListStore *store;
        store=(GtkListStore*)gtk_tree_view_get_model((GtkTreeView*)palView);	
		GtkTreeIter iter;
		gtk_list_store_append(store,&iter);
		gtk_list_store_set (store,&iter,0,string,1,"",2,"",3,"",-1);
		for (pa=pal->begin();pa!=pal->end();pa++)
			{
			char mv[10],rel[10];
			sprintf(mv,"%f",(*pa).second->get_membership_value());
			sprintf(rel,"%f",(*pa).second->get_reliability_value());
			GtkTreeIter iter;
			gtk_list_store_append(store,&iter);
			gtk_list_store_set (store,&iter,0,(*pa).second->get_name(),
											1,(*pa).second->get_label(),
											2,mv,
											3,rel,-1);
			}	
		}
		
	// 6) Riempio la Action List
	al=fuz->get_action_list();
	if (al)
		{
		char string[20];
		sprintf(string,"Mr. Brian->run(%d)",it);
		GtkWidget *alView=lookup_widget((GtkWidget*)button,"actview");
		GtkListStore *store;
        store=(GtkListStore*)gtk_tree_view_get_model((GtkTreeView*)alView);	
		GtkTreeIter iter;
		gtk_list_store_append(store,&iter);
		gtk_list_store_set (store,&iter,0,string,1,"",2,"",3,"",-1);
		for (a=al->begin();a!=al->end();a++)
			{
			char mv[10],rel[10];
			sprintf(mv,"%f",(*a).second->get_membership_value());
			sprintf(rel,"%f",(*a).second->get_reliability_value());
			GtkTreeIter iter;
			gtk_list_store_append(store,&iter);
			gtk_list_store_set (store,&iter,0,(*a).second->get_name(),
											1,(*a).second->get_label(),
											2,mv,
											3,rel,-1);
			}				
		}
		
	//fine riempimento liste
		
	gtk_widget_show(crispList);
	gtk_widget_show(fuzzyList);
	gtk_widget_show(wwantList);
	gtk_widget_show(cmdList);
	gtk_widget_show(predList);
	}
else displayMessage((GtkWidget*)button,user_data,"Sorry Mr. Brian doesn't exist, you're stupid!");
}





void on_resetBrian_clicked (GtkButton *button,gpointer user_data)
{
GtkWidget *entry;
GtkWidget *window;
GtkWidget *status;
char string[7];
	
window=lookup_widget(GTK_WIDGET(button),"mainWindow");

it=0;
//dealloco tutti i puntatori di brian e/o liste esisenti...
if (brian_the_brain) { free((void*)brian_the_brain); brian_the_brain=NULL; }
if (crispList) { gtk_widget_destroy(crispList); crispList=NULL; }
if (fuzzyList) { gtk_widget_destroy(fuzzyList); fuzzyList=NULL; }
if (wwantList) { gtk_widget_destroy(wwantList); wwantList=NULL; }
if (cmdList)   { gtk_widget_destroy(cmdList); cmdList=NULL; }
if (predList)  { gtk_widget_destroy(predList); predList=NULL; }
if (palList)  { gtk_widget_destroy(palList); palList=NULL; }
if (actList)  { gtk_widget_destroy(actList); actList=NULL; }
GtkWidget *item1=lookup_widget((GtkWidget*)button,"crisp_data_list1");
GtkWidget *item2=lookup_widget((GtkWidget*)button,"fuzzy_data_list1");
GtkWidget *item3=lookup_widget((GtkWidget*)button,"weight_want_list1");
GtkWidget *item4=lookup_widget((GtkWidget*)button,"command_list1");
GtkWidget *item5=lookup_widget((GtkWidget*)button,"predicate_list1");
GtkWidget *item6=lookup_widget((GtkWidget*)button,"addCrisp");
GtkWidget *item7=lookup_widget((GtkWidget*)button,"runBrian");
GtkWidget *item10=lookup_widget((GtkWidget*)button,"resetBrian");
GtkWidget *item8=lookup_widget((GtkWidget*)button,"proposed_action_list1");
GtkWidget *item9=lookup_widget((GtkWidget*)button,"action_list1");
GtkWidget *item11=lookup_widget((GtkWidget*)button,"hide_all");
GtkWidget *item12=lookup_widget((GtkWidget*)button,"show_all");
gtk_check_menu_item_set_active((GtkCheckMenuItem*)item1,FALSE);
gtk_check_menu_item_set_active((GtkCheckMenuItem*)item2,FALSE);
gtk_check_menu_item_set_active((GtkCheckMenuItem*)item3,FALSE);
gtk_check_menu_item_set_active((GtkCheckMenuItem*)item4,FALSE);
gtk_check_menu_item_set_active((GtkCheckMenuItem*)item5,FALSE);
gtk_check_menu_item_set_active((GtkCheckMenuItem*)item8,FALSE);
gtk_check_menu_item_set_active((GtkCheckMenuItem*)item9,FALSE);
gtk_widget_set_sensitive(item1,FALSE);
gtk_widget_set_sensitive(item2,FALSE);
gtk_widget_set_sensitive(item3,FALSE);
gtk_widget_set_sensitive(item4,FALSE);
gtk_widget_set_sensitive(item5,FALSE);
gtk_widget_set_sensitive(item6,FALSE);
gtk_widget_set_sensitive(item7,FALSE);
gtk_widget_set_sensitive(item8,FALSE);
gtk_widget_set_sensitive(item9,FALSE);
gtk_widget_set_sensitive(item10,FALSE);
gtk_widget_set_sensitive(item11,FALSE);
gtk_widget_set_sensitive(item12,FALSE);
}

void on_crisp_data_list_toggle(GtkMenuItem *item, gpointer user_data)
{
if (brian_the_brain)
	{
	if (!crispList) createCrispDataList((GtkWidget*)item,user_data);
	if (GTK_WIDGET_VISIBLE(crispList))
		{
		gtk_widget_hide(crispList);
		gtk_check_menu_item_set_active((GtkCheckMenuItem*)item,FALSE);
		}
	else if (!GTK_WIDGET_VISIBLE(crispList))
		{
		gtk_check_menu_item_set_active((GtkCheckMenuItem*)item,TRUE);
		gtk_widget_show(crispList);
		}
	}
else g_printerr("\nThere's no Mr.Brian\n");
}


void on_fuzzy_data_list_toggle (GtkMenuItem *item, gpointer user_data)
{
if (!fuzzyList) createFuzzyDataList((GtkWidget*)item,user_data);
if (fuzzyList)
	{
	if (GTK_WIDGET_VISIBLE(fuzzyList))
		{
		gtk_widget_hide(fuzzyList);
		gtk_check_menu_item_set_active((GtkCheckMenuItem*)item,FALSE);
		}
	else if (!GTK_WIDGET_VISIBLE(fuzzyList))
		{
		gtk_check_menu_item_set_active((GtkCheckMenuItem*)item,TRUE);
		gtk_widget_show(fuzzyList);
 		}
	}
else gtk_check_menu_item_set_active((GtkCheckMenuItem*)item,FALSE);
}

void on_weight_want_list_toggle (GtkMenuItem *item, gpointer user_data)
{
if (!wwantList) createWeightWantList((GtkWidget*)item,user_data);
if (wwantList)
	{
	if (GTK_WIDGET_VISIBLE(wwantList))
		{
		gtk_widget_hide(wwantList);
		gtk_check_menu_item_set_active((GtkCheckMenuItem*)item,FALSE);
		}
	else if (!GTK_WIDGET_VISIBLE(wwantList))
		{
		gtk_check_menu_item_set_active((GtkCheckMenuItem*)item,TRUE);
		gtk_widget_show(wwantList);
		}
	}
else g_printerr("\nThere's no Mr.Brian\n");
}


void on_command_list_toggle (GtkMenuItem *item, gpointer user_data)
{
if (!cmdList) createCommandList((GtkWidget*)item,user_data);
if (cmdList)
	{
	if (GTK_WIDGET_VISIBLE(cmdList))
		{
		gtk_widget_hide(cmdList);
		gtk_check_menu_item_set_active((GtkCheckMenuItem*)item,FALSE);
		}
	else if (!GTK_WIDGET_VISIBLE(cmdList))
		{
		gtk_check_menu_item_set_active((GtkCheckMenuItem*)item,TRUE);
		gtk_widget_show(cmdList);
		}
	}
else g_printerr("\nThere's no Mr.Brian\n");
}
void on_predicate_list_toggle (GtkMenuItem *item, gpointer user_data)
{
if (!predList) createPredicateList((GtkWidget*)item,user_data);
if (predList)
	{
	if (GTK_WIDGET_VISIBLE(predList))
		{
		gtk_widget_hide(predList);
		gtk_check_menu_item_set_active((GtkCheckMenuItem*)item,FALSE);
		}
	else if (!GTK_WIDGET_VISIBLE(predList))
		{
		gtk_check_menu_item_set_active((GtkCheckMenuItem*)item,TRUE);
		gtk_widget_show(predList);
		}
	}
else g_printerr("\nThere's no Mr.Brian\n");
}




void on_proposed_action_list_toggle(GtkMenuItem *item, gpointer user_data)
{
if (!palList) createPalList((GtkWidget*)item,user_data);
if (palList)
	{
	if (GTK_WIDGET_VISIBLE(palList))
		{
		gtk_widget_hide(palList);
		gtk_check_menu_item_set_active((GtkCheckMenuItem*)item,FALSE);
		}
	else if (!GTK_WIDGET_VISIBLE(palList))
		{
		gtk_check_menu_item_set_active((GtkCheckMenuItem*)item,TRUE);
		gtk_widget_show(palList);
		}
	}
else g_printerr("\nThere's no Mr.Brian\n");
}

void on_action_list_toggle(GtkMenuItem *item, gpointer user_data)
{
if (!actList) createActList((GtkWidget*)item,user_data);
if (actList)
	{
	if (GTK_WIDGET_VISIBLE(actList))
		{
		gtk_widget_hide(actList);
		gtk_check_menu_item_set_active((GtkCheckMenuItem*)item,FALSE);
		}
	else if (!GTK_WIDGET_VISIBLE(actList))
		{
		gtk_check_menu_item_set_active((GtkCheckMenuItem*)item,TRUE);
		gtk_widget_show(actList);
		}
	}
else g_printerr("\nThere's no Mr.Brian\n");
}

void on_hide_all_clicked(GtkMenuItem *item, gpointer user_data)
{
GtkWidget *item1=lookup_widget((GtkWidget*)item,"crisp_data_list1");
GtkWidget *item2=lookup_widget((GtkWidget*)item,"fuzzy_data_list1");
GtkWidget *item3=lookup_widget((GtkWidget*)item,"weight_want_list1");
GtkWidget *item4=lookup_widget((GtkWidget*)item,"command_list1");
GtkWidget *item5=lookup_widget((GtkWidget*)item,"predicate_list1");
GtkWidget *item6=lookup_widget((GtkWidget*)item,"proposed_action_list1");
GtkWidget *item7=lookup_widget((GtkWidget*)item,"action_list1");
if (crispList) gtk_check_menu_item_set_active((GtkCheckMenuItem*)item1,FALSE);
if (fuzzyList) gtk_check_menu_item_set_active((GtkCheckMenuItem*)item2,FALSE);
if (wwantList) gtk_check_menu_item_set_active((GtkCheckMenuItem*)item3,FALSE);
if (cmdList) gtk_check_menu_item_set_active((GtkCheckMenuItem*)item4,FALSE);
if (predList) gtk_check_menu_item_set_active((GtkCheckMenuItem*)item5,FALSE);
if (palList) gtk_check_menu_item_set_active((GtkCheckMenuItem*)item6,FALSE);
if (actList) gtk_check_menu_item_set_active((GtkCheckMenuItem*)item7,FALSE);
}

void on_show_all_clicked(GtkMenuItem *item, gpointer user_data)
{
GtkWidget *item1=lookup_widget((GtkWidget*)item,"crisp_data_list1");
GtkWidget *item2=lookup_widget((GtkWidget*)item,"fuzzy_data_list1");
GtkWidget *item3=lookup_widget((GtkWidget*)item,"weight_want_list1");
GtkWidget *item4=lookup_widget((GtkWidget*)item,"command_list1");
GtkWidget *item5=lookup_widget((GtkWidget*)item,"predicate_list1");
GtkWidget *item6=lookup_widget((GtkWidget*)item,"proposed_action_list1");
GtkWidget *item7=lookup_widget((GtkWidget*)item,"action_list1");
if (crispList) gtk_check_menu_item_set_active((GtkCheckMenuItem*)item1,TRUE);
if (fuzzyList) gtk_check_menu_item_set_active((GtkCheckMenuItem*)item2,TRUE);
if (wwantList) gtk_check_menu_item_set_active((GtkCheckMenuItem*)item3,TRUE);
if (cmdList) gtk_check_menu_item_set_active((GtkCheckMenuItem*)item4,TRUE);
if (predList) gtk_check_menu_item_set_active((GtkCheckMenuItem*)item5,TRUE);
if (palList) gtk_check_menu_item_set_active((GtkCheckMenuItem*)item6,TRUE);
if (actList) gtk_check_menu_item_set_active((GtkCheckMenuItem*)item7,TRUE);
gtk_widget_hide(crispList); gtk_widget_show(crispList);
gtk_widget_hide(fuzzyList); gtk_widget_show(fuzzyList);
gtk_widget_hide(wwantList); gtk_widget_show(wwantList);
gtk_widget_hide(cmdList);   gtk_widget_show(cmdList);
gtk_widget_hide(predList);  gtk_widget_show(predList);
gtk_widget_hide(palList);   gtk_widget_show(palList);
gtk_widget_hide(actList);   gtk_widget_show(actList);
}


void on_back_to_main_clicked(GtkWidget *widget, gpointer user_data)
{
g_print("\nOn back to main clicked");
gtk_widget_show(mainWin);
gtk_widget_hide(windowino);
}
void on_hide_clicked(GtkWidget *widget, gpointer user_data)
{
g_print("\nOn hide clicked");
GtkWidget *window=lookup_widget(widget,"mainWindow");
mainWin=window;
gtk_widget_hide(window);
if (!windowino)
	{
	windowino=gtk_window_new(GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title ((GtkWindow*)windowino,"Little Brian");
	gtk_window_resize((GtkWindow*)windowino,20,20);
	gtk_window_move((GtkWindow*)windowino,0,0);	
	GtkWidget *backBox=gtk_vbox_new(TRUE,0);
	gtk_container_add((GtkContainer*)windowino,backBox);
	GtkWidget *backButton=gtk_button_new_with_label("Back");
	gtk_box_pack_start(GTK_BOX(backBox),backButton,FALSE,FALSE,0);
	gtk_widget_show(backButton);
	gtk_widget_show(backBox);
	gtk_widget_show(windowino);
	
	g_signal_connect(windowino,"delete_event",G_CALLBACK(on_back_to_main_clicked),NULL);
	g_signal_connect(windowino,"delete_event",G_CALLBACK(gtk_widget_hide_on_delete),NULL);
	g_signal_connect(backButton,"clicked",G_CALLBACK(on_back_to_main_clicked),NULL);
	}
else gtk_widget_show(windowino);
}


void on_del_crisp_clicked(GtkWidget *widget, gpointer user_data)
{
//elimino dato crisp corrispondente al tasto del premuto...
GtkWidget *parentTab=gtk_widget_get_parent(widget);
GList *last=g_list_last(gtk_container_get_children((GtkContainer*)parentTab));
if (last)
	{
	GtkWidget *entry=(GtkWidget*)last->data;
	const gchar *label=gtk_entry_get_text((GtkEntry*)entry);
	crisp_data *data=cdl->get_by_name(label);
    cdl->del(cdl->get_by_name(label));
	gtk_widget_destroy(parentTab);
	}
else g_printerr("\nErrore non trovo l'entry");
}

//****************************************** METODI DI SERVIZIO *****************************
void crisp_hide()
{
GtkWidget *item=lookup_widget(crispList,"crispitem");
gtk_check_menu_item_set_active((GtkCheckMenuItem*)item,FALSE);
}

void fuzzy_hide(void)
{
GtkWidget *item=lookup_widget(fuzzyList,"fuzzyitem");
gtk_check_menu_item_set_active((GtkCheckMenuItem*)item,FALSE);
}
void wwant_hide(void)
{
GtkWidget *item=lookup_widget(wwantList,"wwantitem");
gtk_check_menu_item_set_active((GtkCheckMenuItem*)item,FALSE);
}
void cmd_hide(void)
{
GtkWidget *item=lookup_widget(cmdList,"cmditem");
gtk_check_menu_item_set_active((GtkCheckMenuItem*)item,FALSE);
}
void pred_hide(void)
{
GtkWidget *item=lookup_widget(predList,"preditem");
gtk_check_menu_item_set_active((GtkCheckMenuItem*)item,FALSE);
}

void pal_hide(void)
{
GtkWidget *item=lookup_widget(palList,"palitem");
gtk_check_menu_item_set_active((GtkCheckMenuItem*)item,FALSE);
}

void act_hide(void)
{
GtkWidget *item=lookup_widget(actList,"actitem");
gtk_check_menu_item_set_active((GtkCheckMenuItem*)item,FALSE);
}

void createCrispDataList(GtkWidget *widget, gpointer user_data)
{
GtkWidget *window=lookup_widget(GTK_WIDGET(widget),"mainWindow");
GtkWidget *scroll;
GtkWidget *cbox;
GtkWidget *nameLabel;
GtkWidget *valueLabel;
GtkWidget *relLabel;
GtkWidget *tab;

crispList=gtk_window_new(GTK_WINDOW_TOPLEVEL);
gtk_window_set_title ((GtkWindow*)crispList,"Crisp Data List");
gtk_window_set_default_size((GtkWindow*)crispList,538,218);
gtk_window_move((GtkWindow*)crispList,476,0);

scroll=gtk_scrolled_window_new(NULL,NULL);
gtk_container_add((GtkContainer*)crispList,scroll);
cbox=gtk_vbox_new(FALSE,0);
gtk_scrolled_window_add_with_viewport((GtkScrolledWindow*)scroll,cbox);
nameLabel=gtk_entry_new();
gtk_entry_set_text((GtkEntry*)nameLabel,"DATA NAME");
gtk_entry_set_editable((GtkEntry*)nameLabel,FALSE);
valueLabel=gtk_entry_new();
gtk_entry_set_text((GtkEntry*)valueLabel,"DATA VALUE");
gtk_entry_set_editable((GtkEntry*)valueLabel,FALSE);
relLabel=gtk_entry_new();
gtk_entry_set_text((GtkEntry*)relLabel,"RELIABILITY");
gtk_entry_set_editable((GtkEntry*)relLabel,FALSE);


tab=gtk_table_new(1,4,FALSE);
gtk_box_pack_start(GTK_BOX(cbox),tab,FALSE,FALSE,0);
gtk_table_attach (GTK_TABLE (tab),nameLabel,0,1,0,1,(GtkAttachOptions)(GTK_FILL),(GtkAttachOptions)(0),0,0);
gtk_table_attach (GTK_TABLE (tab),valueLabel,1,2,0,1,(GtkAttachOptions)(GTK_FILL),(GtkAttachOptions)(0),0,0);
gtk_table_attach (GTK_TABLE (tab),relLabel,2,3,0,1,(GtkAttachOptions)(GTK_FILL),(GtkAttachOptions)(0),0,0);

gtk_widget_show(cbox);
gtk_widget_show(scroll);
gtk_widget_show(nameLabel);
gtk_widget_show(valueLabel);
gtk_widget_show(relLabel);
gtk_widget_show(tab);

GLADE_HOOKUP_OBJECT (window, cbox, "crispbox");
GtkWidget *item=lookup_widget(widget,"crisp_data_list1");
GLADE_HOOKUP_OBJECT (crispList, item, "crispitem");
g_signal_connect ((gpointer)crispList,"delete_event",G_CALLBACK(crisp_hide),NULL);
g_signal_connect ((gpointer) crispList,"delete_event",G_CALLBACK(gtk_widget_hide_on_delete),NULL);
}

void createFuzzyDataList(GtkWidget *widget, gpointer user_data)
{
GtkWidget *window=lookup_widget(GTK_WIDGET(widget),"mainWindow");
GtkWidget *scroll;
GtkWidget *fbox;

if (brian_the_brain)
	{
	fuzzyList=gtk_window_new(GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title ((GtkWindow*)fuzzyList,"Fuzzy Data List");
	gtk_window_set_default_size((GtkWindow*)fuzzyList,390,218);
	gtk_window_move((GtkWindow*)fuzzyList,69,0);
	scroll=gtk_scrolled_window_new(NULL,NULL);
	gtk_container_add((GtkContainer*)fuzzyList,scroll);
	fbox=gtk_vbox_new(FALSE,0);
	gtk_scrolled_window_add_with_viewport((GtkScrolledWindow*)scroll,fbox);
	gtk_widget_show(fbox);
	gtk_widget_show(scroll);
		
	GtkWidget *view;
	GtkTreeViewColumn *col;
	GtkCellRenderer *renderer;
		
	view=gtk_tree_view_new();
	renderer=gtk_cell_renderer_text_new();
	gtk_tree_view_insert_column_with_attributes(GTK_TREE_VIEW(view),-1,"Name",renderer,"text",0,NULL);
	renderer=gtk_cell_renderer_text_new();
	gtk_tree_view_insert_column_with_attributes(GTK_TREE_VIEW(view),-1,"Label",renderer,"text",1,NULL);
	renderer=gtk_cell_renderer_text_new();
	gtk_tree_view_insert_column_with_attributes(GTK_TREE_VIEW(view),-1,"Membership",renderer,"text",2,NULL);
	gtk_container_add(GTK_CONTAINER(fbox),view);
	gtk_widget_show(view);
		
	GtkListStore *store;
	store=gtk_list_store_new(3,G_TYPE_STRING,G_TYPE_STRING,G_TYPE_STRING);	
  	gtk_tree_view_set_model (GTK_TREE_VIEW(view),GTK_TREE_MODEL(store));	
	
	GLADE_HOOKUP_OBJECT (window, view, "fuzzyview");
	GtkWidget *item=lookup_widget(widget,"fuzzy_data_list1");
	GLADE_HOOKUP_OBJECT (fuzzyList, item, "fuzzyitem");
	
	g_signal_connect ((gpointer)fuzzyList,"delete_event",G_CALLBACK(fuzzy_hide),NULL);
	g_signal_connect ((gpointer)fuzzyList,"delete_event",G_CALLBACK(gtk_widget_hide_on_delete),NULL);
	}
else g_printerr("\nThere's no Mr.Brian\n");
}

void createWeightWantList(GtkWidget *widget, gpointer user_data)
{
GtkWidget *window=lookup_widget(GTK_WIDGET(widget),"mainWindow");
GtkWidget *scroll;
GtkWidget *wbox;

if (brian_the_brain)
	{
	wwantList=gtk_window_new(GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title ((GtkWindow*)wwantList,"Weight Want List");
	gtk_window_set_default_size((GtkWindow*)wwantList,237,223);
	gtk_window_move((GtkWindow*)wwantList,521,509);
	scroll=gtk_scrolled_window_new(NULL,NULL);
	gtk_container_add((GtkContainer*)wwantList,scroll);
	wbox=gtk_vbox_new(FALSE,0);
	gtk_scrolled_window_add_with_viewport((GtkScrolledWindow*)scroll,wbox);
	gtk_widget_show(wbox);
	gtk_widget_show(scroll);

	GtkWidget *view;
	GtkTreeViewColumn *col;
	GtkCellRenderer *renderer;
	
	view=gtk_tree_view_new();
	renderer=gtk_cell_renderer_text_new();
	gtk_tree_view_insert_column_with_attributes(GTK_TREE_VIEW(view),-1,"Want name",renderer,"text",0,NULL);
	renderer=gtk_cell_renderer_text_new();
	gtk_tree_view_insert_column_with_attributes(GTK_TREE_VIEW(view),-1,"Value",renderer,"text",1,NULL);
	gtk_container_add(GTK_CONTAINER(wbox),view);
	gtk_widget_show(view);
	
	GtkListStore *store;
	store=gtk_list_store_new(3,G_TYPE_STRING,G_TYPE_STRING,G_TYPE_STRING);	
  	gtk_tree_view_set_model (GTK_TREE_VIEW(view),GTK_TREE_MODEL(store));	
	
	GLADE_HOOKUP_OBJECT (window, view, "wwview");
	GtkWidget *item=lookup_widget(widget,"weight_want_list1");
	GLADE_HOOKUP_OBJECT (wwantList, item, "wwitem");
	
	g_signal_connect ((gpointer)wwantList,"delete_event",G_CALLBACK(wwant_hide),NULL);
	g_signal_connect ((gpointer)wwantList,"delete_event",G_CALLBACK(gtk_widget_hide_on_delete),NULL);
	}
else g_printerr("\nThere's no Mr.Brian\n");
}
void createCommandList(GtkWidget *widget, gpointer user_data)
{
GtkWidget *window=lookup_widget(GTK_WIDGET(widget),"mainWindow");
GtkWidget *scroll;
GtkWidget *cmbox;

if (brian_the_brain)
	{
	cmdList=gtk_window_new(GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title ((GtkWindow*)cmdList,"Command List");
	gtk_window_set_default_size((GtkWindow*)cmdList,255,222);
	gtk_window_move((GtkWindow*)cmdList,760,509);
	scroll=gtk_scrolled_window_new(NULL,NULL);
	gtk_container_add((GtkContainer*)cmdList,scroll);
	cmbox=gtk_vbox_new(FALSE,0);
	gtk_scrolled_window_add_with_viewport((GtkScrolledWindow*)scroll,cmbox);
	gtk_widget_show(cmbox);
	gtk_widget_show(scroll);
	
	GtkWidget *view;
	GtkTreeViewColumn *col;
	GtkCellRenderer *renderer;
	
	view=gtk_tree_view_new();
	renderer = gtk_cell_renderer_text_new ();
	gtk_tree_view_insert_column_with_attributes(GTK_TREE_VIEW(view),-1,"Command",renderer,"text",0,NULL);
	renderer = gtk_cell_renderer_text_new ();
	gtk_tree_view_insert_column_with_attributes(GTK_TREE_VIEW(view),-1,"Set Point",renderer,"text",1,NULL);
	gtk_container_add (GTK_CONTAINER (cmbox), view);
	gtk_widget_show(view);
	
	
    GtkListStore *store;
    store=gtk_list_store_new(3,G_TYPE_STRING,G_TYPE_STRING,G_TYPE_STRING);	
  	gtk_tree_view_set_model (GTK_TREE_VIEW(view),GTK_TREE_MODEL(store));		
	
	GLADE_HOOKUP_OBJECT (window, view, "cmview");
	GtkWidget *item=lookup_widget(widget,"command_list1");
	GLADE_HOOKUP_OBJECT (cmdList, item, "cmditem");
	
	g_signal_connect ((gpointer)cmdList,"delete_event",G_CALLBACK(cmd_hide),NULL);
	g_signal_connect ((gpointer) cmdList,"delete_event",G_CALLBACK(gtk_widget_hide_on_delete),NULL);
	}
else g_printerr("\nThere's no Mr.Brian\n");
}
void createPredicateList(GtkWidget *widget, gpointer user_data)
{
GtkWidget *window=lookup_widget(GTK_WIDGET(widget),"mainWindow");
GtkWidget *scroll;
GtkWidget *pbox;

if (brian_the_brain)
	{
	predList=gtk_window_new(GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title ((GtkWindow*)predList,"Predicate List");
	gtk_window_set_default_size((GtkWindow*)predList,517,222);
	gtk_window_move((GtkWindow*)predList,1,509);
	scroll=gtk_scrolled_window_new(NULL,NULL);
	gtk_container_add((GtkContainer*)predList,scroll);
	pbox=gtk_vbox_new(FALSE,0);
	gtk_scrolled_window_add_with_viewport((GtkScrolledWindow*)scroll,pbox);
	gtk_widget_show(pbox);
	gtk_widget_show(scroll);
	
	GtkWidget *view;
	GtkTreeViewColumn *col;
	GtkCellRenderer *renderer;
	
	view=gtk_tree_view_new();
	renderer = gtk_cell_renderer_text_new ();
	gtk_tree_view_insert_column_with_attributes(GTK_TREE_VIEW(view),-1,"Predicate",renderer,"text",0,NULL);
	renderer = gtk_cell_renderer_text_new ();
	gtk_tree_view_insert_column_with_attributes(GTK_TREE_VIEW(view),-1,"Membership",renderer,"text",1,NULL);
	renderer = gtk_cell_renderer_text_new ();
	gtk_tree_view_insert_column_with_attributes(GTK_TREE_VIEW(view),-1,"Reliability",renderer,"text",2,NULL);
	gtk_container_add (GTK_CONTAINER (pbox), view);
	gtk_widget_show(view);
	
	
    GtkListStore *store;
    store=gtk_list_store_new(3,G_TYPE_STRING,G_TYPE_STRING,G_TYPE_STRING);	
  	gtk_tree_view_set_model (GTK_TREE_VIEW(view),GTK_TREE_MODEL(store));	
	
	GLADE_HOOKUP_OBJECT (window, view, "pview");
	GtkWidget *item=lookup_widget(widget,"predicate_list1");
	GLADE_HOOKUP_OBJECT (predList, item, "preditem");
	
	g_signal_connect ((gpointer)predList,"delete_event",G_CALLBACK(pred_hide),NULL);
	g_signal_connect ((gpointer) predList,"delete_event",G_CALLBACK(gtk_widget_hide_on_delete),NULL);
	}
else g_printerr("\nThere's no Mr.Brian\n");
}





void createPalList(GtkWidget *widget, gpointer user_data)
{
GtkWidget *window=lookup_widget(GTK_WIDGET(widget),"mainWindow");
GtkWidget *scroll;
GtkWidget *palbox;

if (brian_the_brain)
	{
	palList=gtk_window_new(GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title ((GtkWindow*)palList,"Proposed Action List");
	gtk_window_set_default_size((GtkWindow*)palList,520,236);
	gtk_window_move((GtkWindow*)palList,494,245);
	scroll=gtk_scrolled_window_new(NULL,NULL);
	gtk_container_add((GtkContainer*)palList,scroll);
	palbox=gtk_vbox_new(FALSE,0);
	gtk_scrolled_window_add_with_viewport((GtkScrolledWindow*)scroll,palbox);
	gtk_widget_show(palbox);
	gtk_widget_show(scroll);
	
	GtkWidget *view;
	GtkTreeViewColumn *col;
	GtkCellRenderer *renderer;
	
	view=gtk_tree_view_new();
	renderer = gtk_cell_renderer_text_new ();
	gtk_tree_view_insert_column_with_attributes(GTK_TREE_VIEW(view),-1,"Action Name",renderer,"text",0,NULL);
	renderer = gtk_cell_renderer_text_new ();
	gtk_tree_view_insert_column_with_attributes(GTK_TREE_VIEW(view),-1,"Action Label",renderer,"text",1,NULL);
	renderer = gtk_cell_renderer_text_new ();
	gtk_tree_view_insert_column_with_attributes(GTK_TREE_VIEW(view),-1,"Membership",renderer,"text",2,NULL);
	renderer = gtk_cell_renderer_text_new ();
	gtk_tree_view_insert_column_with_attributes(GTK_TREE_VIEW(view),-1,"Reliability",renderer,"text",3,NULL);
	gtk_container_add (GTK_CONTAINER (palbox), view);
	gtk_widget_show(view);
	
	
    GtkListStore *store;
    store=gtk_list_store_new(4,G_TYPE_STRING,G_TYPE_STRING,G_TYPE_STRING,G_TYPE_STRING);	
  	gtk_tree_view_set_model (GTK_TREE_VIEW(view),GTK_TREE_MODEL(store));
	
	GLADE_HOOKUP_OBJECT (window, view, "palview");
	GtkWidget *item=lookup_widget(widget,"proposed_action_list1");
	GLADE_HOOKUP_OBJECT (palList, item, "palitem");
	
	g_signal_connect ((gpointer)palList,"delete_event",G_CALLBACK(pal_hide),NULL);
	g_signal_connect ((gpointer) palList,"delete_event",G_CALLBACK(gtk_widget_hide_on_delete),NULL);
	}
else g_printerr("\nThere's no Mr.Brian\n");
}

void createActList(GtkWidget *widget, gpointer user_data)
{
GtkWidget *window=lookup_widget(GTK_WIDGET(widget),"mainWindow");
GtkWidget *scroll;
GtkWidget *actbox;

if (brian_the_brain)
	{
	actList=gtk_window_new(GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title ((GtkWindow*)actList,"Action List");
	gtk_window_set_default_size((GtkWindow*)actList,488,237);
	gtk_window_move((GtkWindow*)actList,1,245);
	scroll=gtk_scrolled_window_new(NULL,NULL);
	gtk_container_add((GtkContainer*)actList,scroll);
	actbox=gtk_vbox_new(FALSE,0);
	gtk_scrolled_window_add_with_viewport((GtkScrolledWindow*)scroll,actbox);
	gtk_widget_show(actbox);
	gtk_widget_show(scroll);
	
	GtkWidget *view;
	GtkTreeViewColumn *col;
	GtkCellRenderer *renderer;
	
	view=gtk_tree_view_new();
	renderer = gtk_cell_renderer_text_new ();
	gtk_tree_view_insert_column_with_attributes(GTK_TREE_VIEW(view),-1,"Action Name",renderer,"text",0,NULL);
	renderer = gtk_cell_renderer_text_new ();
	gtk_tree_view_insert_column_with_attributes(GTK_TREE_VIEW(view),-1,"Action Label",renderer,"text",1,NULL);
	renderer = gtk_cell_renderer_text_new ();
	gtk_tree_view_insert_column_with_attributes(GTK_TREE_VIEW(view),-1,"Membership",renderer,"text",2,NULL);
	renderer = gtk_cell_renderer_text_new ();
	gtk_tree_view_insert_column_with_attributes(GTK_TREE_VIEW(view),-1,"Reliability",renderer,"text",3,NULL);
	gtk_container_add (GTK_CONTAINER (actbox), view);
	gtk_widget_show(view);
	
	
    GtkListStore *store;
    store=gtk_list_store_new(4,G_TYPE_STRING,G_TYPE_STRING,G_TYPE_STRING,G_TYPE_STRING);	
  	gtk_tree_view_set_model (GTK_TREE_VIEW(view),GTK_TREE_MODEL(store));
	
	GLADE_HOOKUP_OBJECT (window, view, "actview");
	GtkWidget *item=lookup_widget(widget,"action_list1");
	GLADE_HOOKUP_OBJECT (actList, item, "actitem");
	
	g_signal_connect ((gpointer)actList,"delete_event",G_CALLBACK(act_hide),NULL);
	g_signal_connect ((gpointer) actList,"delete_event",G_CALLBACK(gtk_widget_hide_on_delete),NULL);
	}
else g_printerr("\nThere's no Mr.Brian\n");
}

//ritorna TRUE se la creazione di brian ha avuto successo
gint createBrian(GtkWidget *widget, gpointer user_data)
{
//prima recupero tutti i nomi dei files di configurazione da caricare
if (brian_the_brain) return 1;
GtkEntry *entry1,*entry2,*entry3,*entry4,*entry5,*entry6,*entry7,*entry8,*entry9; //per recuperare caselle di testo
char *fassoc,*fshape,*dfassoc,*dfshape,*want,*cando,*pries,*actpries,*beh;
entry1=(GtkEntry*)lookup_widget(GTK_WIDGET(widget),"entry1");//fuzzyassoc
entry2=(GtkEntry*)lookup_widget(GTK_WIDGET(widget),"entry2");//fuzzyshapes
entry3=(GtkEntry*)lookup_widget(GTK_WIDGET(widget),"entry3");//defuzzyassoc
entry4=(GtkEntry*)lookup_widget(GTK_WIDGET(widget),"entry4");//defuzzyshapes
entry5=(GtkEntry*)lookup_widget(GTK_WIDGET(widget),"entry5");//behaviours
entry6=(GtkEntry*)lookup_widget(GTK_WIDGET(widget),"entry6");//cando
entry7=(GtkEntry*)lookup_widget(GTK_WIDGET(widget),"entry7");//want
entry8=(GtkEntry*)lookup_widget(GTK_WIDGET(widget),"entry8");//pries
entry9=(GtkEntry*)lookup_widget(GTK_WIDGET(widget),"entry9");//actionpries

fassoc=(char*)gtk_entry_get_text(entry1);
fshape=(char*)gtk_entry_get_text(entry2);
dfassoc=(char*)gtk_entry_get_text(entry3);
dfshape=(char*)gtk_entry_get_text(entry4);
beh=(char*)gtk_entry_get_text(entry5);
cando=(char*)gtk_entry_get_text(entry6);
want=(char*)gtk_entry_get_text(entry7);
pries=(char*)gtk_entry_get_text(entry8);
actpries=(char*)gtk_entry_get_text(entry9);	
brian_the_brain=new MrBrian(fassoc,fshape,pries,actpries,cando,beh,want,dfassoc,dfshape);
fuz=(*brian_the_brain).getFuzzy();
cdl=(*fuz).get_crisp_data_list();
if (brian_the_brain) return 0;
else return -1;
}


//ritorna il nome del file selezionato da un FileChooser
const gchar *get_filename(GtkWidget *widget, gpointer user_data, const gchar *title)
{
const gchar *f=NULL;
const gchar *buf=NULL;
GtkWidget *window=lookup_widget(GTK_WIDGET(widget),"mainWindow");
GtkWidget *fileselector=gtk_file_selection_new("Select Config File");
gtk_window_set_title((GtkWindow*)fileselector,title);
if (gtk_dialog_run(GTK_DIALOG(fileselector))==GTK_RESPONSE_OK)
	buf=gtk_file_selection_get_filename((GtkFileSelection*)fileselector);
g_print("\nF: %s Buf: %s",f,buf);
if (buf) 
	{
	f=(const gchar*)g_malloc(sizeof(char)*(strlen(buf)+1));
	memcpy((void*)f,buf,strlen(buf)+1);
	g_print("\nF: %s Buf: %s",f,buf);
	g_print("\nstrlen(F)= %d",strlen(f));
	}
gtk_widget_destroy(fileselector);
if (f) return f;
else return NULL;
}

//visualizza un messaggio di informazione...
void displayMessage(GtkWidget *widget, gpointer user_data, const gchar *message)
{
GtkWidget *window=lookup_widget(GTK_WIDGET(widget),"mainWindow");
GtkWidget *messageDialog=gtk_message_dialog_new((GtkWindow*)window,GTK_DIALOG_DESTROY_WITH_PARENT,GTK_MESSAGE_INFO,GTK_BUTTONS_OK,message);
if (gtk_dialog_run(GTK_DIALOG(messageDialog))==GTK_RESPONSE_OK)
	gtk_widget_destroy(messageDialog);
}
