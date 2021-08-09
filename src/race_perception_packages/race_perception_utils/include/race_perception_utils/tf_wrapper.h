#ifndef _TF_WRAPPER_
#define _TF_WRAPPER_


/* _________________________________
   |                                 |
   |           INCLUDES              |
   |_________________________________| */

//boost includes

//ROS includes
#include <ros/ros.h>
#include <tf/transform_listener.h>

//PCL includes

//race includes

//raceua includes

//Namespaces
using namespace std;

namespace race_perception_utils
{

    bool safe_tf_wait(boost::shared_ptr<tf::TransformListener> p_tf_listener, string target, string source, int times_to_wait_for_three_secs, std::string name="unknown")
    {
       //Query time (using ros::time::now() and wait until time is different from
        //zero, since this is what causes the problem
        ros::Rate loop_rate(20);
        bool flag=true;
        while (flag)
        {
            ros::Time t_test = ros::Time::now();
            if (t_test.toSec()!=0)
            {
                flag=false;
            }
            else
            {
                loop_rate.sleep(); //sleep
                //ROS_INFO("Waiting for propper ros::Time:::now() call");
            }
        }

        //now wait for first transform, weird that first call never works, I guess
        //some old fashion sleep is needed, probably because the first
        //ros::Time::now() (although not zero) is still very early and the publisher
        //had not started yet
        bool got_transform=false;

        ros::Time start_tic = ros::Time::now();
        ros::Time time = ros::Time::now();
        int count=0;

        while (ros::ok() && !got_transform && count < times_to_wait_for_three_secs)
        {
            try
            {
                //ROS_INFO_STREAM("Waiting for transform at time " << time << " (count " << count << ")");
                got_transform = p_tf_listener->waitForTransform(target , source, time, ros::Duration(3), ros::Duration(0.01));
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
            }

            count++;
            loop_rate.sleep(); //sleep
            //ros::spinOnce(); // Handle ROS events
            time = ros::Time::now();
        }

        if (!got_transform)
        {
            ROS_ERROR("%s: Could not get transform from %s to %s after waiting %f secs", name.c_str(), target.c_str(), source.c_str(), (ros::Time::now() - start_tic).toSec());
            return false;
        }
        else
        {
            ROS_INFO("%s: Got transform from %s to %s after waiting %f secs", name.c_str(), target.c_str(), source.c_str(), (ros::Time::now() - start_tic).toSec());
        }

        return true;
    }


    bool safe_tf_listen(boost::shared_ptr<tf::TransformListener> p_tf_listener, tf::StampedTransform* transf, string target, string source, ros::Time time)
    {
        //Query time (using ros::time::now() and wait until time is different from
        //zero, since this is what causes the problem
        ros::Rate loop_rate(20);
//         bool flag=true;
//         while (flag)
//         {
//             ros::Time t_test = ros::Time::now();
//             if (t_test.toSec()!=0)
//             {
//                 flag=false;
//             }
//             else
//             {
//                // loop_rate.sleep(); //sleep
//                 //ROS_INFO("Waiting for propper ros::Time:::now() call");
//             }
//         }

        //now wait for first transform, weird that first call never works, I guess
        //some old fashion sleep is needed, probably because the first
        //ros::Time::now() (although not zero) is still very early and the publisher
        //had not started yet
        bool got_transform=false;
        ros::Time t = ros::Time::now();
        int count=0;
        while (ros::ok() && !got_transform && count<3)
        {
            try
            {
                //ROS_INFO_STREAM("Waiting for transform at time " << time << " (count " << count << ")");
                got_transform = p_tf_listener->waitForTransform(target , source, time, ros::Duration(0.1), ros::Duration(0.01));
		break;
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
            }

            count++;
            loop_rate.sleep(); //sleep
            //ros::spinOnce(); // Handle ROS events
        }

        if (!got_transform)
        {
            ROS_WARN("Could not get transform from %s to %s after waiting %f secs", target.c_str(), source.c_str(), (ros::Time::now() - t).toSec());
            return false;
        }
        //else
            //ROS_INFO("Got transform after waiting %f secs", (ros::Time::now() - t).toSec());

        //Finally now we can lookup transforms without fearing exceptions
        //To test for expections you can comment any of the two cycles above and you
        //should start to see some of the exceptions
        try
        {
            //ros::Time now = ros::Time::now();
            //ROS_INFO_STREAM("Looking up transform at time " << time);
            p_tf_listener->lookupTransform(target, source, time, *transf);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }


        return true;
    }


}//end race_perception_utils namespace
#endif
