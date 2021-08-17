#!/bin/bash
   
#Release Date: 1, Sep, 2020

echo "-------- [Step-0] update bashrc by adding student_ws as ros workspace --------"

#0- Setup .bashrc by adding student_ws as ros workspace
cd ~/

echo  "

#Add ROS cognitive_robotics_ws as workspace
source /opt/ros/melodic/setup.bash
source $HOME/cognitive_robotics_ws/devel/setup.bash
export LD_LIBRARY_PATH=/opt/ros/melodic/lib:\${LD_LIBRARY_PATH:+:\${LD_LIBRARY_PATH}}
export ROS_PACKAGE_PATH=\$HOME/cognitive_robotics_ws:/opt/ros/melodic/include:\${ROS_PACKAGE_PATH}

export ROBOT=sim
export PATH=~/bin:\$PATH

export PATH=/usr/lib/python2.7/:\$PATH
export LC_NUMERIC=\"en_US.UTF-8\" 

#This line forces the system to use python2 by default since the ROS works based on python2
alias python='/usr/bin/python2.7'" >> ~/.bashrc


