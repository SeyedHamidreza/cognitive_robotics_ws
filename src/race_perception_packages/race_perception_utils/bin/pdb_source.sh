#!/bin/bash

if [ -z "$1" ]
then
    echo "No argument supplied"
    exit
fi

rm -rf /tmp/pdb
mkdir -p /tmp/pdb
. $ROS_ROOT/../rosbash/rosbash
#. /opt/ros/fuerte/share/rosbash/rosbash
roscd race_perception_db
cd pdb

cp -Rf $1/* /tmp/pdb/

