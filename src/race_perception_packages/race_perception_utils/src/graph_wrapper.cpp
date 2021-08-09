/**
  \file
  \brief GVGraph class implementation code
  */

#include <race_perception_utils/graph_wrapper.h>

Agnode_t*GVGraph::selectNode(string&name)
{
	map<string, Agnode_t*>::iterator it = _nodes.find(name);
	if(it!=_nodes.end())
		return it->second;
	else
		return NULL;
}

Agedge_t*GVGraph::selectEdge(string&source,string&target)
{
	pair<string, string> key=make_pair<string,string>(source,target);

	if(_edges.find(key)!=_edges.end())
		return _edges[key];

	return NULL;
}

int GVGraph::setGraphAttribute(string attribute,string value)
{
	//printf("line=%d\n",__LINE__);

	return _agset(_graph, attribute, value);
}

int GVGraph::setNodeAttribute(string name,string attribute,string value)
{
	//printf("line=%d\n",__LINE__);
	char* l = agstrdup_html( (value.c_str()));

	//agset (n,"label",l);

	agsafeset(selectNode(name),const_cast<char*>(attribute.c_str()),l,const_cast<char*>(""));

    agstrfree (l);
	//printf("line=%d\n",__LINE__);
	return 1;
	// 	return _agset(selectNode(name),attribute,value);
	//
	return agsafeset(selectNode(name),const_cast<char*>(attribute.c_str()),const_cast<char*>(value.c_str()),const_cast<char*>(""));
}

int GVGraph::setEdgeAttribute(string source,string target,string attribute,string value)
{
	return agsafeset(selectEdge(source,target),const_cast<char*>(attribute.c_str()),const_cast<char*>(value.c_str()),const_cast<char*>(""));

	// 	return _agset(selectEdge(source,target),attribute,value);
}

void GVGraph::startRender()
{
	gvRender(_context,_graph,(char*)"xgtk",NULL);
}

void GVGraph::startRenderFP(FILE *out)
{
	gvRender(_context,_graph,(char*)"dot",out);
}

void GVGraph::startRenderFile(char *filename)
{
	//printf("line=%d\n",__LINE__);
	gvRenderFilename(_context,_graph,(char*)"dot",filename);
}

GVC_t*GVGraph::getGVCcontext(void)
{
	return _context;
}

Agraph_t* GVGraph::_agopen(string name,int kind)
{
	return agopen(const_cast<char*>(name.c_str()),kind);
}

string GVGraph::_agget(void*object,string attr,string alt)
{
	string str=agget(object, const_cast<char*>(attr.c_str()));

	if(str==string())
		return alt;
	else
		return str;
}

int GVGraph::_agset(void*object,string attr,string value)
{
	return agsafeset(object, const_cast<char*>(attr.c_str()),const_cast<char*>(value.c_str()),const_cast<char*>(value.c_str()));
}

Agsym_t* GVGraph::_agnodeattr(string name,string value)
{
	return agnodeattr(_graph,const_cast<char*>(name.c_str()),const_cast<char*>(value.c_str()));
}

Agsym_t* GVGraph::_agedgeattr(string name,string value)
{
	return agedgeattr(_graph,const_cast<char*>(name.c_str()),const_cast<char*>(value.c_str()));
}
Agnode_t* GVGraph::_agnode(string name)
{
	return agnode(_graph,const_cast<char*>(name.c_str()));
}

int GVGraph::_gvLayout(GVC_t*gvc,graph_t*graph,string engine)
{
	return gvLayout(gvc,graph,engine.c_str());
}

void GVGraph::_gvconfig_plugin_install_from_library(GVC_t*gvc, char *path, gvplugin_library_t *library)
{
    gvconfig_plugin_install_from_library(gvc, path, library);
}

GVGraph::GVGraph(string name,double node_size):
	_context(gvContext()),
	_graph(_agopen(name, AGRAPH))
	//_graph(_agopen(name, AGDIGRAPHSTRICT)) // Strict directed graph, see libgraph doc
	//_graph(_agopen(name, AGGRAPH)) // Strict directed graph, see libgraph doc
	//_graph(_agopen(name, METAGRAPH)) // Strict directed graph, see libgraph doc
{
	////Set graph attributes
	////     _agset(_graph, "overlap", "prism");
	////     _agset(_graph, "splines", "true");
	////     _agset(_graph, "pad", "0,2");
	////     _agset(_graph, "dpi", "96,0");
	//_agset(_graph, "nodesep", "0.3");
	//_agset(_graph, "splines", "spline");
	//_agset(_graph, "imagepath", "img");
	//// 	_agset(_graph, "ranksep", "2.0");
	//// 	_agset(_graph, "landscape","true");
	//// 	_agset(_graph, "center","true");
	//// 	_agset(_graph, "aspect","1");
	//// 	_agset(_graph, "forcelabels","true");
	//_agset(_graph, "rankdir","TB");
	////_agset(_graph, "rankdir","TB");
	//// 	_agset(_graph, "style","filled");
	//_agset(_graph, "rankdir","LR");
	_agset(_graph, "rankdir","TB");

	////Set default attributes for the future nodes
	//_agnodeattr("shape", "box");
	//_agnodeattr("height", "0.02");
	_agnodeattr("shape", "box");
	_agnodeattr("height", "0.02");


	////_agnodeattr(_graph, "label", "");
	////_agnodeattr( "regular", "true");

	////Divide the wanted width by the DPI to get the value in points
	//// 	double dpi=lexical_cast<double>(_agget(_graph, "dpi", "96,0"));
	////     string nodePtsWidth= lexical_cast<string>(node_size/dpi);

	////GV uses , instead of . for the separator in floats
	//// 	replace(nodePtsWidth.begin(), nodePtsWidth.end(), '.', ',');

	//// 	cout<<"nodePtsWidth:"<<nodePtsWidth<<endl;
	////     _agnodeattr(_graph, "width", nodePtsWidth);
	////

    ////printf("line %d",__LINE__);
	////char* l;
	////l= agstrdup_html("< <BR> OLA </BR> >");

	////agset(_graph,"label",l);
	////ROS_INFO("line %d",__LINE__);
	////graph_context->setNodeAttribute("Object Detection", "label",agstrdup_html("<<BR> Isto é Bold </BR>>"));
	////ROS_INFO("line %d",__LINE__);


	//printf("line %d",__LINE__);
}

GVGraph::~GVGraph()
{
	gvFreeLayout(_context, _graph);
	agclose(_graph);
	gvFreeContext(_context);
}

void GVGraph::addNode(const string& name)
{
	if(_nodes.find(name)!=_nodes.end())
		removeNode(name);

	_nodes.insert(pair<string,Agnode_t*>(name, _agnode(name)));
}

void GVGraph::addNodes(vector<string>& names)
{
	for(uint i=0; i<names.size(); ++i)
		addNode(names[i]);
}

void GVGraph::removeNode(const string& name)
{
	string nn=name;

	if(_nodes.find(name)!=_nodes.end())
	{
		agdelete(_graph, _nodes[name]);
		_nodes.erase(name);

		map< pair<string,string>, Agedge_t*>::iterator it;

		for(it=_edges.begin();it!=_edges.end();it++)
		{
			if(it->first.first==nn || it->first.second==nn)
				removeEdge(it->first.first,it->first.second);
		}
	}
}

void GVGraph::clearNodes()
{
	map<string, Agnode_t*>::iterator it;

	vector<string>names;

	for(it=_nodes.begin();it!=_nodes.end();it++)
		names.push_back(it->first);

	for(uint i=0;i<names.size();i++)
		removeNode(names[i]);
}

void GVGraph::setRootNode(const string& name)
{
	if(_nodes.find(name)!=_nodes.end())
		_agset(_graph, "root", name);
}

void GVGraph::addEdge(const string &source, const string &target)
{
	if(_nodes.find(source)!=_nodes.end() && _nodes.find(target)!=_nodes.end())
	{
        //printf("find source: %s and target: %s", source.c_str(), target.c_str());

		pair<string,string> key(source,target);

		if(_edges.find(key)==_edges.end())
			_edges.insert( pair< pair<string,string >, Agedge_t* >(key, agedge(_graph, _nodes[source], _nodes[target])));
        else
            printf("Already edge exists between source: %s and target: %s", source.c_str(), target.c_str());
	}
    else
    {
        printf("Can't find source: %s or target: %s", source.c_str(), target.c_str());
    }
}

void GVGraph::removeEdge(const string &source, const string &target)
{
	removeEdge(pair<string,string>(source, target));
}

void GVGraph::removeEdge(const pair<string, string>& key)
{
	if(_edges.find(key)!=_edges.end())
	{
		agdelete(_graph, _edges[key]);
		_edges.erase(key);
	}
}

void GVGraph::freeLayout()
{
	gvFreeLayout(_context, _graph);
}

void GVGraph::applyLayout()
{
	_gvLayout(_context, _graph, "dot");
}
