#ifndef _PRINT_H_
#define _PRINT_H_

/* _________________________________
   |                                 |
   |           INCLUDES              |
   |_________________________________| */

//boost includes
#include <boost/lexical_cast.hpp>

//pstreams includes
#include <pstreams/pstream.h>

//ROS includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>

//PCL includes

//race includes

//raceua includes

//Namespaces
using namespace std;

#define NO_ROSOUT 1

#define TAB "\t"

#define FGBROWN "\033[0;33m"
#define FGBLUE "\033[0;34m"
#define FGGREEN "\033[0;32m"
#define FGBLACK "\033[0;30m"
#define FGCYAN "\033[0;36m"
#define FGYELLOW "\033[1;33m"
#define FGRED "\033[0;31m"
#define FGPURPLE "\033[0;35m"
#define FGLGRAY "\033[0;37m"
#define FGDGRAY "\033[1;30m"
#define FGLBLUE "\033[1;34m"
#define FGLGREEN "\033[1;32m"
#define FGLCYAN "\033[1;36m"
#define FGLRED "\033[1;31m"
#define FGLPURPLE "\033[1;35m"
#define FGDEFAULT "\033[0;39m"
#define BGDEFAULT "\033[49m"
#define UNDERLINE "\033[4m"

#define FGUBLUE "\033[5;34m"

namespace race_perception_utils
{
	/* _________________________________
	   |                                 |
	   |        Class definition         |
	   |_________________________________| */

	class PrettyPrint 
	{
		public:
			//local variables
			string _name;	
			//string _out;
			ostringstream _out;
			std::vector< boost::shared_ptr<string > > _custom;
			std::vector< int > _custom_type;
			ros::WallTime t;

			/* _________________________________
			   |                                 |
			   |           CONSTRUCTORS          |
			   |_________________________________| */

			PrettyPrint()
			{
				t = ros::WallTime::now();
                string name = ros::this_node::getName();
                setName(name);
				//_p_n = n;	
			}

			PrettyPrint(std::string name)
			{
				t = ros::WallTime::now();
                setName(name);
				//_p_n = n;	
			}

			//PrettyPrint(){setName("unknown name");};
			//PrettyPrint(string s){setName(s);};
			~PrettyPrint(){};

			void setName(std::string s)
			{
				_name = s;
			}
			/* _________________________________
			   |                                 |
			   |           CLASS METHODS         |
			   |_________________________________| */

			void warn(std::ostream& ss)
			{
				boost::shared_ptr<string> s = (boost::shared_ptr<string>)new string;
				*s = dynamic_cast<ostringstream&>(ss).str();
				_custom.push_back(s);
				_custom_type.push_back(2);
				return;
			}

			void error(std::ostream& ss)
			{
				boost::shared_ptr<string> s = (boost::shared_ptr<string>)new string;
				*s = dynamic_cast<ostringstream&>(ss).str();
				_custom.push_back(s);
				_custom_type.push_back(1);
				return;
			}

			void info(std::ostream& ss)
			{
				boost::shared_ptr<string> s = (boost::shared_ptr<string>)new string;
				*s = dynamic_cast<ostringstream&>(ss).str();
				_custom.push_back(s);
				_custom_type.push_back(0);
				return;
			}
			

			void printCallback()
			{
                string name = _name;
                    //name = ros::this_node::getName();
				//string ns = ros::this_node::getNamespace();
				//vector<string> publishers;
				//ros::this_node::getAdvertisedTopics(publishers);
				//vector<string> subscribers;
				//ros::this_node::getSubscribedTopics(subscribers);
				ros::WallDuration d = ros::WallTime::now() - t;

				cout << endl;
				_out << FGBLUE << name << FGDEFAULT << " callback (" << d.toSec() << " secs):" << endl;

				for (size_t i=0; i< _custom.size(); ++i)
				{
					if (_custom_type[i]==0)
					{
						_out << TAB << FGDEFAULT <<*_custom[i] << FGDEFAULT<< endl;
					}
					else if (_custom_type[i]==1)
					{
						_out << TAB << "Error: " << FGRED <<*_custom[i] << FGDEFAULT << endl;
					}
					else if (_custom_type[i]==2)
					{
						_out << TAB << "Warn: " << FGBROWN <<*_custom[i] << FGDEFAULT << endl;
					}
				}
				ROS_INFO_STREAM((_out.str()).c_str());

			}

			void printInitialization()
			{
				//vector<string> publishers = _p_n->getAdvertisedTopics();
				//ros::this_node::g
                string name = _name;

				//string name = ros::this_node::getName();
				//string ns = ros::this_node::getNamespace();
				vector<string> publishers;
			    ros::this_node::getAdvertisedTopics(publishers);
				vector<string> subscribers;
			    ros::this_node::getSubscribedTopics(subscribers);

				std::string inst;
				inst = "rosservice list " + name;

				//print names of all header files in current directory
				redi::ipstream in(inst.c_str());
				std::string str;
				vector<string> services;
				while (std::getline(in, str)) 
				{
					services.push_back(str);
				}

				cout << endl;
				_out << FGUBLUE << name << FGDEFAULT << " initialized:" << endl;
				//_out << "Namespace "<< FGDGRAY << ns << FGDEFAULT << endl;

				_out << "Published topics (" << publishers.size() << "):" <<endl;
				for (size_t i=0; i< publishers.size(); ++i)
				{
					#if NO_ROSOUT
					std::size_t found = publishers[i].find("rosout");
					if (found!=std::string::npos)
					{}
					else
					{
					#endif
						_out << TAB << FGDGRAY << publishers[i] << FGDEFAULT << endl;
					#if NO_ROSOUT
					}
					#endif
				}

				_out << "Subscribed topics (" << subscribers.size() << "):" <<endl;
				for (size_t i=0; i< subscribers.size(); ++i)
					_out << TAB << FGDGRAY << subscribers[i] << FGDEFAULT << endl;

				_out << "Provided services (" << services.size() << "):" <<endl;
				for (size_t i=0; i< services.size(); ++i)
				{
					#if NO_ROSOUT
					std::size_t found = services[i].find("get_loggers");
					std::size_t found1 = services[i].find("set_logger_level");
					if (found!=std::string::npos || found1!=std::string::npos)
					{}
					else
					{
					#endif
						_out << TAB << FGDGRAY << services[i] << FGDEFAULT << endl;
					#if NO_ROSOUT
					}
					#endif

				}

				_out << "Custom prints:" <<endl;
				for (size_t i=0; i< _custom.size(); ++i)
				{
					if (_custom_type[i]==0)
					{
						_out << TAB << FGDGRAY <<*_custom[i] << FGDEFAULT<< endl;
					}
					else if (_custom_type[i]==1)
					{
						_out << TAB << "Error: " << FGRED <<*_custom[i] << FGDEFAULT << endl;
					}
					else if (_custom_type[i]==2)
					{
						_out << TAB << "Warn: " << FGBROWN <<*_custom[i] << FGDEFAULT << endl;
					}
				}

				ROS_INFO_STREAM((_out.str()).c_str());
			}



	};



}//end race_perception_utils namespace
#endif
