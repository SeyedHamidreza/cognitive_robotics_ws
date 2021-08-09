#ifndef _CYCLE_H_
#define _CYCLE_H_

/* _________________________________
   |                                 |
   |           INCLUDES              |
   |_________________________________| */

//boost includes
#include <boost/lexical_cast.hpp>

//ROS includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
//#include <std_msgs/Float32.h>
#include <race_perception_utils/Cycle.h>
#include <race_perception_utils/CallbackTime.h>

//PCL includes

//race includes

//raceua includes

//Namespaces
using namespace std;

namespace race_perception_utils
{
	/* _________________________________
	   |                                 |
	   |        Class definition         |
	   |_________________________________| */

	class CycleDebug 
	{
		public:
			//local variables
			string _name;	
			boost::shared_ptr<ros::Publisher> _p_publisher; 
			boost::shared_ptr<ros::Publisher> _p_publisher1; 
			ros::WallTime t;
			ros::WallTime wall_tic;
            ros::Time sim_tic;
            size_t count;
            std::vector<double> _durations;

			/* _________________________________
			   |                                 |
			   |           CONSTRUCTORS          |
			   |_________________________________| */

			CycleDebug(ros::NodeHandle* n, std::string name)
			{
				_name = name;
                string topic = "/perception/cycle";

				//initialize the publisher
				_p_publisher = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
				*_p_publisher = n->advertise<race_perception_utils::Cycle>(topic,100);

                //initialize the publisher
				_p_publisher1 = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
				*_p_publisher1 = n->advertise<race_perception_utils::CallbackTime>("/perception/callback_time",100);

				//t = ros::Time::now();
				//ros::Duration(0.05).sleep();
				t = ros::WallTime::now();
				wall_tic = ros::WallTime::now();
                count = 0;

			}

			//CycleDebug(ros::NodeHandle* n)
			//{
				//CycleDebug(*n, ros::this_node::getName());
			//}

			~CycleDebug()
			{
				_p_publisher.reset(); 
			};


			void tic(void) //to be called at the start of the callback
            {
				wall_tic = ros::WallTime::now();
                sim_tic = ros::Time::now();
            }


			void run(void) //to be called at the end of the callback
			{

				race_perception_utils::CallbackTime msg1;
                msg1.tic = sim_tic;
                msg1.toc = ros::Time::now();
                msg1.duration = (ros::WallTime::now() - wall_tic).toSec();
                msg1.name = _name;
                _p_publisher1->publish(msg1);

                return;
				race_perception_utils::Cycle msg;
				ros::WallDuration d = ros::WallTime::now() - t;

                //compute the cycle time and add to vector
				ros::WallDuration wall_d = ros::WallTime::now() - wall_tic;
                _durations.push_back(wall_d.toSec());

				if (d.toSec()>=4.0)
				{
                    //compute the frequency of calls
					msg.frequency = (float)count / d.toSec(); //get the frequency
					msg.name = _name; 
					count = 0;
                    msg.header.stamp = ros::Time::now();

                    //compute the average cycle duration
                    if (_durations.size()==0)
                    {
                        msg.avg_duration = 0;
                    }
                    else
                    {
                        double total;
                        for (size_t i=0; i<_durations.size(); i++)
                        {
                            total += _durations[i];
                        }
                        msg.avg_duration = total/_durations.size();
                    }

                    //publish the message
					_p_publisher->publish(msg);

                    t = ros::WallTime::now(); //reset the tic

                    _durations.erase(_durations.begin(), _durations.end()); //erase all durations

				}
				else
				{
					count++;	
				}
			}

			/* _________________________________
			   |                                 |
			   |           CLASS METHODS         |
			   |_________________________________| */

	};



}//end race_perception_utils namespace
#endif
