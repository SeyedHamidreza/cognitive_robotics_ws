/**
\file
\brief Callback code for the xgtk plug-in
*/

#include <gtk/gtk.h>

#include <graphviz/gvplugin_device.h>
#include <graphviz/gvplugin_render.h>
#include <graphviz/gvplugin.h>
#include <graphviz/gvc.h>
#include <graphviz/gvplugin_layout.h>
#include <graphviz/gvcjob.h>
#include <graphviz/gvcommon.h>
#include <graphviz/gvcext.h>
#include <graphviz/types.h>
//#include <graphviz/graph.h>
#include <graphviz/geom.h>

#include "callbacks.h"//silly prototypes, this include is useless

#include <iostream>

#include <boost/format.hpp>

//#include <mtt/hypothesisTree.h>

#include <race_perception_utils/graph_wrapper.h>

using namespace std;
using namespace boost;

///External variable linking to the main graph context created in the mht main source code pmht.cpp
extern GVGraph*graph_context;

extern pthread_mutex_t _draw_mutex;
///External variable linking to the Mht context class (main workhorse of the mht algorithm)
//extern hypothesisTreePtr htreePtr;

extern "C" {
	extern int agrelabel_node(Agnode_t * n, char *newname);
}

ostream& operator<< (ostream &o, const pointf &i)
{
	return o<<i.x<<" "<<i.y;
}

ostream& operator<< (ostream &o, const boxf &i)
{
	return o<<"LL: "<<i.LL<<" UR: "<<i.UR;
}


gboolean drawGraph(void)
{
	if(pthread_mutex_trylock(&(_draw_mutex))!=0)//its locked
		return false;
	
	//if(htreePtr->need_layout==false)
	//{
		//pthread_mutex_unlock(&(htreePtr->_draw_mutex));
		//return false;
	//}

	graph_context->freeLayout();

	//HypothesisTree::iterator parent;
	//HypothesisTree::iterator it;
	
	//graph_context->clearNodes();
		
	////graph_context=new GVGraph("graph 123");
	////gvconfig_plugin_install_from_library(graph_context->getGVCcontext(), NULL, &gvplugin_xgtk_LTX_library);

	//graph_context->addNode("node1");
	//graph_context->setRootNode("node1");
	//graph_context->addNode("node2");
	//graph_context->addNode("node3");
	//graph_context->addEdge("node1", "node3");

	graph_context->applyLayout();

	//ROS_INFO("in thread");
	//graph_context->startRender();

	//for(it = htreePtr->begin(); it != htreePtr->end(); ++it)
	//{
		//assert(*it!=0);
		
		//graph_context->addNode((*it)->nameUI());

		//for(uint a=0;a<(*it)->_attribute_names.size();++a)
			//graph_context->setNodeAttribute((*it)->nameUI(),(*it)->_attribute_names[a],(*it)->_attribute_values[a]);
		
		//parent=htreePtr->parent(it);

		//if(parent!=NULL)
			//graph_context->addEdge((*parent)->nameUI(),(*it)->nameUI());
	//}

	//graph_context->applyLayout();
	
	//htreePtr->need_layout=false;
	
	pthread_mutex_unlock(&(_draw_mutex));
	
	return true;
}

gboolean DrawingareaExposeEvent(GtkWidget*widget,GdkEventExpose*event,gpointer user_data)
{	
     //cout<<"in expose"<<endl;
	pthread_mutex_lock(&(_draw_mutex));
	
	GVJ_t *job = (GVJ_t *)g_object_get_data(G_OBJECT(widget),"job");
	cairo_t *cr = gdk_cairo_create(widget->window);
	
	(job->callbacks->motion)(job, job->pointer);

	job->context = (void *)cr;
	job->external_context = true;
	job->width = widget->allocation.width;
	job->height = widget->allocation.height;
	
// 	cout<<"refresh"<<endl;
	(job->callbacks->refresh)(job);
	
// 	cout<<"destroy"<<endl;
	cairo_destroy(cr);
	
	pthread_mutex_unlock(&(_draw_mutex));
// 	cout<<"out of DrawingareaExposeEvent"<<endl;
	return true;
}

gboolean DrawingareaButtonPressEvent(GtkWidget*widget,GdkEventButton *event,gpointer user_data)
{
	static bool init=true;
	static bool dragging=false;
	
	if(init)
	{
		g_object_set_data(G_OBJECT(widget),"dragging",&dragging);
		init=false;
	}
	
	if(event->type==GDK_BUTTON_PRESS)
		dragging=true;
	else if(event->type==GDK_BUTTON_RELEASE)
		dragging=false;
	
	return true;
}

gboolean DrawingareaMotionNotifyEvent(GtkWidget*widget,GdkEventMotion*event,gpointer user_data)
{
	static bool first_drag=true;
	
	GVJ_t *job = (GVJ_t *)g_object_get_data(G_OBJECT(widget),"job");
	
	bool *dragging_p = (bool*)g_object_get_data(G_OBJECT(widget),"dragging");
	if(!dragging_p)
		return FALSE;//the draging variable was not been set yet, it is set by the button press event handler
	
	bool dragging=*dragging_p;
	
	static pointf diff;
	
	double dpix=-1/job->devscale.x;//can also be interpreted as scroling speed
	double dpiy=-1/job->devscale.y;//can also be interpreted as scroling speed
		
	diff.x=event->x-job->pointer.x;
	diff.y=event->y-job->pointer.y;
	
	job->pointer.x = event->x;
	job->pointer.y = event->y;
	
	gtk_widget_queue_draw(widget);

	if(dragging)
	{
		if(first_drag)
		{
			diff.x=0;
			diff.y=0;
			first_drag=false;
		}

		job->focus.x+= diff.x*dpix/job->zoom;
		job->focus.y+= diff.y*dpiy/job->zoom;
		
		double x_lim = (0.37*job->width-1)/job->zoom;
		double y_lim = (0.37*job->height-1)/job->zoom;
		
// 		cout<<"Dpi: "<<job->dpi<<endl;
// 		cout<<"View: "<<job->view<<endl;
// 		cout<<"translation: "<<job->translation<<endl;
// 		cout<<"devscale: "<<job->devscale<<endl;
// 		cout<<"focus: "<<job->focus<<endl;
// 		cout<<"scale: "<<job->scale<<endl;
		
		if(job->focus.x>= x_lim)
			job->focus.x= x_lim;
		
		if(job->focus.y>= y_lim)
			job->focus.y= y_lim;
	}
	
    return true;
}

gboolean DrawingareaScrollEvent(GtkWidget*widget,GdkEventScroll *event,gpointer user_data)
{
	if(event->type==GDK_SCROLL)
	{
		GVJ_t *job = (GVJ_t *)g_object_get_data(G_OBJECT(widget),"job");
		
		pointf pointer;
		pointer.x=event->x;
		pointer.y=event->y;
		
		if(event->direction==GDK_SCROLL_UP)
			(job->callbacks->button_press)(job,4, pointer);
		else if(event->direction==GDK_SCROLL_DOWN)
			(job->callbacks->button_press)(job,5, pointer);

		double x_lim = (0.37*job->width-1)/job->zoom;
		double y_lim = (0.37*job->height-1)/job->zoom;
		
		if(job->focus.x>= x_lim)
			job->focus.x= x_lim;
		
		if(job->focus.y>= y_lim)
			job->focus.y= y_lim;
	}
	
	return true;
}
                                                        
gboolean WindowDeleteEvent(GtkWidget*widget,GdkEvent*event,gpointer user_data)
{
	printf("delete event");
    gtk_main_quit();
    return true;
}

gboolean DrawingareaConfigureEvent(GtkWidget*widget,GdkEventConfigure*event,gpointer user_data)
{
	GVJ_t *job;
	double zoom_to_fit;

/*FIXME - should allow for margins */
/*      - similar zoom_to_fit code exists in: */
/*      plugin/gtk/callbacks.c */
/*      plugin/xlib/gvdevice_xlib.c */
/*      lib/gvc/gvevent.c */

    job = (GVJ_t *)g_object_get_data(G_OBJECT(widget),"job");
	
    if (! job->has_been_rendered)
	{
		zoom_to_fit = MIN((double) event->width / (double) job->width,(double) event->height / (double) job->height);
        if (zoom_to_fit < 1.0) /* don't make bigger */
			job->zoom *= zoom_to_fit;
    }else if(job->fit_mode)
	{
		zoom_to_fit = MIN((double) event->width / (double) job->width, (double) event->height / (double) job->height);
		job->zoom *= zoom_to_fit;
    }
    
    if ((unsigned int)event->width > job->width || (unsigned int)event->height > job->height)
		job->has_grown = TRUE;
    
	job->width = event->width;
    job->height = event->height;
    job->needs_refresh = TRUE;

    return FALSE;
}
