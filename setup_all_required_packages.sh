#!/bin/bash
   
#Release Date: 15, Aug, 2021

echo "-------- [Step-0] update bashrc by adding student_ws as ros workspace --------"

#0- Setup .bashrc by adding student_ws as ros workspace
cd ~/

source ~/.bashrc


#1- Setup your sources.list : Setup your computer to accept software from packages.ros.org.
echo "-------- [Step-1] Setup your sources.list --------"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
echo Done
sleep 2


#2- Set up your keys:
echo "-------- [Step-2] Set up your keys --------"

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sleep 2

curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -

echo Done


#Make sure your Debian package index is up-to-date:
sudo apt update
sleep 2
 
#3- Install required packages:
echo "-------- [Step-3] Install required packages --------"

echo "-------- Waiting for dpkg to be free --------"
c=$(ps aux | grep -i apt | wc -l)

spin[0]="-"
spin[1]="\\"
spin[2]="|"
spin[3]="/"

while [[ $c -gt 1 ]]; do
  for i in "${spin[@]}"; do
        echo -ne "\b$i"
        sleep 0.1
  done	
  c=$(ps aux | grep -i apt | wc -l)
done


echo "-------- Ready to install required packages --------"

yes Y | sudo apt-get install ros-melodic-desktop-full ros-melodic-hector-gazebo ros-melodic-openni-camera ros-melodic-roslisp python-rosinstall python-rdflib mercurial git openjdk-11-jdk libcgal-dev libpcl-dev libpstreams-dev libgraphviz-dev python-shapely python-networkx python-nltk python-pip libsnappy-dev ros-melodic-catkin python-catkin-tools libusb-dev libspnav-dev libbluetooth-dev libcwiid-dev ros-melodic-rviz-visual-tools build-essential cmake doxygen libqt4-dev libqt4-opengl-dev libqglviewer-dev-qt4 ros-melodic-moveit rospack-tools python-rosmake pcl-tools

sudo rosdep init
rosdep update

yes Y | sudo apt-get update 
yes Y | sudo apt-get dist-upgrade

yes Y | sudo add-apt-repository ppa:gnome-terminator
yes Y | sudo apt-get update
yes Y | sudo apt-get install terminator


#4- Install KERAS and 
echo "-------- [Step-4] Install KERAS= and TF==1.14.0 --------"

yes Y | sudo pip install --upgrade pip
yes Y | sudo pip install --upgrade protobuf
yes Y | sudo pip install tensorflow==1.14.0
yes Y | sudo pip install keras==2.3.1
yes Y | sudo pip install pytictoc

#5- Install LEVElDB
echo "-------- [Step-4] Install LEVElDB --------"
mkdir -p /tmp/leveldb && cd /tmp/leveldb
wget https://github.com/google/leveldb/archive/v1.14.tar.gz
tar -xzf v1.14.tar.gz
cd leveldb-1.14/
make -j4
sudo cp -R include/leveldb /usr/local/include  
sudo mv libleveldb.* /usr/local/lib
sudo ldconfig


cd ~/

source .bashrc

echo "-------- [Step-5] update and install python --------"

sudo apt-get install software-properties-common
sudo apt-add-repository universe
sudo apt-get update

sudo apt install python3-pip
pip3 install numpy matplotlib pandas seaborn

echo "-------- Installation completed --------"



