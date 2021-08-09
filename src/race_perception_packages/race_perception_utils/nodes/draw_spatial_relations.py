#!/usr/bin/env python
import roslib; roslib.load_manifest('race_msgs')

#from std_msgs.msg import String
from race_msgs.msg import SpatialRelation, SpatialRelationSet, SpatialEntity
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import rospy
import random

pub = rospy.Publisher('spatial_relations_marker', MarkerArray)

def callback(srs):
    rospy.loginfo(rospy.get_name() + ": received SpatialRelationSet msg")

    count = 0
    ma = MarkerArray()
    elist = []
    for r in srs.relations:
        count += 1

        for e1 in srs.entities:
            if e1.name == r.entity1:
                break
        else:
            rospy.logwarn("Could not find entity e1...")
            return

        for e2 in srs.entities:
            if e2.name == r.entity2:
                break
        else:
            rospy.logwarn("Could not find entity e2...")
            return


        epair = [r.entity1, r.entity2]
        epair_swaped = [r.entity2, r.entity1]

        if epair_swaped in elist:
            rospy.loginfo("Relation %d: %s %s %s already exists" % (count,r.entity1, r.relation_type,r.entity2) )

        else:
            elist.append(epair)

            rospy.loginfo("Relation %d: %s (%f; %f; %f) %s %s (%f; %f; %f)" % (count,r.entity1, e1.bbox.pose_stamped.pose.position.x, e1.bbox.pose_stamped.pose.position.y, e1.bbox.pose_stamped.pose.position.z, r.relation_type,r.entity2, e2.bbox.pose_stamped.pose.position.x, e2.bbox.pose_stamped.pose.position.y, e2.bbox.pose_stamped.pose.position.z) )


            rospy.loginfo("elist now is %s" % elist)
            m = Marker()
            m.ns = 'relation_arrow'
            m.lifetime = rospy.Duration.from_sec(15);
            m.id = count
            m.header.frame_id = e1.bbox.pose_stamped.header.frame_id
            m.header.stamp = e1.bbox.pose_stamped.header.stamp
            #m.pose = e1.bbox.pose_stamped.pose
            m.scale.x = 0.005
            m.scale.y = 0.03
            m.scale.z = 0.02
            m.color.r = 0 #random.random()
            m.color.g = 0 #random.random()
            m.color.b = random.random()
            m.color.a = 0.4
            vx = e2.bbox.pose_stamped.pose.position.x - e1.bbox.pose_stamped.pose.position.x
            vy = e2.bbox.pose_stamped.pose.position.y - e1.bbox.pose_stamped.pose.position.y
            vz = e2.bbox.pose_stamped.pose.position.z - e1.bbox.pose_stamped.pose.position.z

            p1 = Point() #just to initializ
            p1.x = e1.bbox.pose_stamped.pose.position.x + vx*0.1
            p1.y = e1.bbox.pose_stamped.pose.position.y + vy*0.1
            p1.z = e1.bbox.pose_stamped.pose.position.z + vz*0.1

            p2 = Point() #just to initialize
            p2.x = e1.bbox.pose_stamped.pose.position.x + vx*0.9
            p2.y = e1.bbox.pose_stamped.pose.position.y + vy*0.9
            p2.z = e1.bbox.pose_stamped.pose.position.z + vz*0.9

            #m.points = [e1.bbox.pose_stamped.pose.position, e2.bbox.pose_stamped.pose.position]
            m.points = [p1, p2]
            #ma.markers = [ma.markers, m]
            ma.markers.append(m)

            m1 = Marker()
            m1.lifetime = rospy.Duration.from_sec(5);
            m1.type = 9
            m1.ns = 'relation_name'
            m1.id = count
            m1.header.frame_id = e1.bbox.pose_stamped.header.frame_id
            m1.header.stamp = e1.bbox.pose_stamped.header.stamp
            m1.pose.position.x = (e1.bbox.pose_stamped.pose.position.x + e2.bbox.pose_stamped.pose.position.x) /2;
            m1.pose.position.y = (e1.bbox.pose_stamped.pose.position.y + e2.bbox.pose_stamped.pose.position.y) /2;
            m1.pose.position.z = (e1.bbox.pose_stamped.pose.position.z + e2.bbox.pose_stamped.pose.position.z) /2 +random.random()/50 ;

            m1.text = "%s" % (r.relation_type)
            #m1.text = "%s %s %s" % (r.entity1, r.relation_type, r.entity2)
            #m.pose = e1.bbox.pose_stamped.pose
            m1.scale.z = 0.02
            m1.color = m.color
            m1.color.a = 1
            ma.markers.append(m1)

    #pub = rospy.Publisher('spatial_relations_marker', MarkerArray)
    #rospy.loginfo("ola %d" % ma.size)
    pub.publish(ma)

def listener():
    rospy.init_node('draw_spatial_relations_node', anonymous=False)
    rospy.Subscriber("/object_anchoring/spatial_relations", SpatialRelationSet, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

