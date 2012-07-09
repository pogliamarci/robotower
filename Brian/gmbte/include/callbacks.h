#include <gtk/gtk.h>


void on_nuovo1_activate(GtkMenuItem *menuitem, gpointer user_data);

void on_apri1_activate (GtkMenuItem *menuitem, gpointer user_data);

void on_salva1_activate (GtkMenuItem *menuitem, gpointer user_data);

void on_salva_come1_activate (GtkMenuItem *menuitem, gpointer user_data);

void on_esci1_activate (GtkMenuItem *menuitem, gpointer user_data);

void on_taglia1_activate (GtkMenuItem *menuitem, gpointer user_data);

void on_copia1_activate (GtkMenuItem *menuitem, gpointer user_data);

void on_incolla1_activate (GtkMenuItem *menuitem, gpointer user_data);

void on_elimina1_activate (GtkMenuItem *menuitem, gpointer user_data);

void on_about1_activate (GtkMenuItem *menuitem, gpointer user_data);

void on_browse1_clicked (GtkButton *button, gpointer user_data);

void on_browse2_clicked (GtkButton *button, gpointer user_data);

void on_browse3_clicked (GtkButton *button, gpointer user_data);

void on_browse4_clicked (GtkButton *button, gpointer user_data);

void on_browse5_clicked (GtkButton *button, gpointer user_data);

void on_browse6_clicked (GtkButton *button, gpointer user_data);

void on_browse7_clicked (GtkButton *button, gpointer  user_data);

void on_browse8_clicked (GtkButton *button, gpointer user_data);

void on_browse9_clicked (GtkButton *button, gpointer user_data);

void on_addCrisp_clicked (GtkButton *button, gpointer user_data);

void on_createBrian_clicked (GtkButton *button,gpointer user_data);

void on_runBrian_clicked (GtkButton *button,gpointer user_data);

void on_resetBrian_clicked (GtkButton *button,gpointer user_data);

void on_okbuttonDialog_clicked (GtkButton *button, gpointer user_data);
													 
void on_crisp_data_list_toggle (GtkMenuItem *item, gpointer user_data);

void on_fuzzy_data_list_toggle (GtkMenuItem *item, gpointer user_data);

void on_weight_want_list_toggle (GtkMenuItem *item, gpointer user_data);
													 
void on_command_list_toggle (GtkMenuItem *item, gpointer user_data);

void on_predicate_list_toggle (GtkMenuItem *item, gpointer user_data);

void on_proposed_action_list_toggle(GtkMenuItem *item, gpointer user_data);

void on_action_list_toggle(GtkMenuItem *item, gpointer user_data);

void on_hide_all_clicked(GtkMenuItem *item, gpointer user_data);

void on_show_all_clicked(GtkMenuItem *item, gpointer user_data);

void on_back_to_main_clicked(GtkWidget *widget, gpointer user_data);

void on_hide_clicked(GtkWidget *widget, gpointer user_data);

void on_del_crisp_clicked(GtkWidget *widget, gpointer user_data);




// ****************************** PROTOTIPI METODI DI SERVIZIO **********************************
void createCrispDataList(GtkWidget *widget, gpointer user_data);
void createFuzzyDataList(GtkWidget *widget, gpointer user_data);
void createWeightWantList(GtkWidget *widget, gpointer user_data);
void createCommandList(GtkWidget *widget, gpointer user_data);
void createPredicateList(GtkWidget *widget, gpointer user_data);
void createPalList(GtkWidget *widget, gpointer user_data);
void createActList(GtkWidget *widget, gpointer user_data);
void crisp_hide(void);
void fuzzy_hide(void);
void wwant_hide(void);
void cmd_hide(void);
void pred_hide(void);
void act_hide(void);


gint createBrian(GtkWidget *widget, gpointer user_data);

const gchar *get_filename(GtkWidget *widget, gpointer user_data, const gchar *title);

void displayMessage(GtkWidget *widget, gpointer user_data, const gchar *message);
