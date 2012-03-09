ifndef MODULE
MODULE=brian
endif

exe_name = gmbte

project_root=../../..
module_root=../..

//DEFS += -D_HAVE_CONFIG_H

ifdef OFFLINE
DEFS += -DOFFLINE
endif

GTKMM_CFLAGS = -I/usr/include/gtkmm-2.0 -I/usr/lib/gtkmm-2.0/include -I/usr/include/gtk-2.0 -I/usr/lib/sigc++-1.2/include -I/usr/include/sigc++-1.2 -I/usr/include/glib-2.0 -I/usr/lib/glib-2.0/include -I/usr/lib/gtk-2.0/include -I/usr/include/pango-1.0 -I/usr/X11R6/include -I/usr/include/freetype2 -I/usr/include/atk-1.0 -I/usr/include/libglade-2.0/

GTKMM_LIBS = gtkmm-2.0 gdkmm-2.0 atkmm-1.0 gtk-x11-2.0 pangomm-1.0 glibmm-2.0 sigc-1.2 gdk-x11-2.0 atk-1.0 gdk_pixbuf-2.0 m pangoxft-1.0 pangox-1.0 pango-1.0 gobject-2.0 gmodule-2.0 dl glib-2.0 glade-2.0

GTK_CFLAGS = -DXTHREADS -D_REENTRANT -DXUSE_MTSAFE_API -I/usr/include/gtk-2.0 -I/usr/lib/gtk-2.0/include -I/usr/X11R6/include -I/usr/include/atk-1.0 -I/usr/include/pango-1.0 -I/usr/include/freetype2 -I/usr/include/freetype2/config -I/usr/include/glib-2.0 -I/usr/lib/glib-2.0/include  

GTK_LIBS = gtk-x11-2.0 gdk-x11-2.0 atk-1.0 gdk_pixbuf-2.0 m pangoxft-1.0 pangox-1.0 pango-1.0 gobject-2.0 gmodule-2.0 dl glib-2.0  

INCLUDE_DIRS += -I../../include $(GTK_CFLAGS)
#LIB_DIRS += 

PROJECT_LIBS = brian fuzzy
SYSTEM_LIBS = $(GTK_LIBS) 

-include ../../../shared/Makefile