#!/bin/bash -i

#rosbag filter $1 $(basename $1 .bag)-no-perception.bag 'topic.find("head_mount_kinect") != -1 or (topic == "/tf" and not m.transforms[0].child_frame_id.startswith("/head_mount_kinect"))'

#rosbag filter input.bag output.bag 'topic == "/camera/image_raw/compressed" or topic == "/scan" or topic == "/timetag" or  topic == "/tf" and m.transforms[0].header.frame_id != "/odom" and m.transforms[0].child_frame_id != "/odom"'

#rosbag filter input.bag output.bag 'topic == "ALL THE TOPICS YOU WANT TO KEEP" or topic == "/tf" and m.transforms[0].header.frame_id != "/base_link" and m.transforms[0].child_frame_id != "/virtual_cam"'

#rosbag filter $1 $(basename $1 .bag)-no-perception.bag "topic.find('head_mount_kinect') != -1 or (topic == '/tf' and not m.transforms[0].child_frame_id.startswith('/head_mount_kinect')) or (topic == '/tf' and not m.transforms[0].child_frame_id.startswith('/perception'))"


rosbag filter $1 $(basename $1 .bag)-no-perception.bag "topic.find('head_mount_kinect') != -1 or (topic == '/tf' and not (m.transforms[0].child_frame_id.startswith('/head_mount_kinect') or m.transforms[0].child_frame_id.startswith('/perception')))"

 #rosbag filter  $1 $(basename $1 .bag)-no-perception.bag "topic == '/tf'" --print="'%s @ %d.%d: %s' % (topic, t.secs, t.nsecs, m.transforms[0].child_frame_id)"

