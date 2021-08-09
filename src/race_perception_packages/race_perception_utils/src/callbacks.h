/**
\file
\brief Callback header, only function prototypes
*/

#ifndef _CALLBACKS_H_
#define _CALLBACKS_H_

#include <gtk/gtk.h>

gboolean DrawingareaExposeEvent(GtkWidget*widget,GdkEventExpose*event,gpointer user_data);
gboolean DrawingareaMotionNotifyEvent(GtkWidget*widget,GdkEventMotion*event,gpointer user_data);
gboolean DrawingareaExposeEvent(GtkWidget*widget,GdkEventExpose*event,gpointer user_data);
gboolean WindowDeleteEvent(GtkWidget*widget,GdkEvent*event,gpointer user_data);
gboolean DrawingareaConfigureEvent(GtkWidget*widget,GdkEventConfigure*event,gpointer user_data);
gboolean DrawingareaButtonPressEvent(GtkWidget*widget,GdkEventButton *event,gpointer user_data);
gboolean DrawingareaScrollEvent(GtkWidget*widget,GdkEventScroll *event,gpointer user_data);
gboolean drawGraph(void);

#endif