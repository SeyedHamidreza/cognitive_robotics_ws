#!/usr/bin/env python

    ##
    # @run script
    # @rosrun race_perception_utils filter4perceptions.py bag_name
    # @
    ##

#__________________________________
#|                                 |
#|           IMPORTS               |
#|_________________________________|


import roslib; roslib.load_manifest('race_perception_utils')
import rosbag
import sys
import rospkg
import rospy

#__________________________________
#|                                 |
#|           GLOBAL VARS           |
#|_________________________________|

_name = 'filter4perception'


# _________________________________
#|                                 |
#|             CLASS               |
#|_________________________________|


class Filter4Perception:


    def run(self, bag_name):
        #r = rospy.Rate(1)
        #r.sleep()
        
        rospy.loginfo("%s: filter for perception %s" % (_name, bag_name))

        #bag = rosbag.Bag(bag_name)
        out_name = 'filtered_' + bag_name
        #outbag = rosbag.Bag(out_name, 'w')

        with rosbag.Bag(out_name, 'w') as outbag:
            for topic, msg, t in rosbag.Bag(bag_name).read_messages():
                if topic.find("head_mount_kinect") != -1:
                    outbag.write(topic, msg, t)
                elif topic == '/tf' and msg.transforms[0].child_frame_id.startswith('/head_mount_kinect'):
                    outbag.write(topic, msg, t)
                elif topic == '/tf' and msg.transforms[0].child_frame_id.startswith('/perception'):
                    outbag.write(topic, msg, t)
                else:
                    rospy.loginfo('filter out this topic: %s' % (topic))

        
        #rospy.spin()

 
    def __init__(self):
        reload(sys)
        sys.setdefaultencoding('utf8')
        #rospy.loginfo(sys.getdefaultencoding())

        rospack = rospkg.RosPack()
        self.dataPath = rospack.get_path('dataset');
        self.runDir = self.dataPath + '/images/'



        #if not os.path.exists('/tmp/plot/'):
            #os.makedirs('/tmp/plot/')


if __name__ == "__main__":
    try:
        rospy.init_node(_name, anonymous=False)
        rospy.myargv(argv=sys.argv)

        if len(sys.argv) != 2:
            rospy.loginfo('usage: rosrun race_perception_utils filter4perceptions.py (your_bag).bag')
            exit()

        f4p = Filter4Perception()
        f4p.run(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
