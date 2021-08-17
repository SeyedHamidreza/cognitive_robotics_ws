## Open-Ended Learning Approaches for 3D Object Recognition

###### [Hamidreza Kasaei](https://hkasaei.github.io/) | [cognitive robotics course](https://rugcognitiverobotics.github.io/) | [assignment description](https://github.com/SeyedHamidreza/cognitive_robotics_ws/blob/main/first_assignment_cognitive_robotics.pdf)
##



## Assignment overview
Cognitive science revealed that humans learn to recognize object categories ceaselessly over time. This ability allowsthem to adapt to new environments,  by enhancing their knowledge from the accumulation of experiences and theconceptualization of new object categories. Takin. In particular, the agent can interact with the g this theory as an inspiration, we seek to create an interactive objectrecognition system that can learn 3D object categories in an open-ended fashion. In this project, “open-ended” implies thatthe set of categories to be learned is not known in advance. The training instances are extracted from on-line experiencesof a robot, and thus become gradually available over time, rather than being completely available at the beginning of thelearning process.Your  goal  for  this  assignment  is  to  implement  an  open-ended  learning  approach for 3D object recognition. 

We break this assignment down into two parts:
1. The first part is about implementing/optimizing offline 3D object recognition systems, which take an object view as input and produces the category label as output (e.g., apple, mug, fork, etc).

2. The second part of this assignment is dedicated to testing your approach in an open-ended fashion.In this assignment, the number of categories is not pre-defined in advance and the knowledge of agent/robot is increasing over time by interacting with a simulated teacher using three actions: teach, ask, and correct (see Fig.1).

<p align="center">
  <img src="images/simulated_user.jpg" width="500" title="">
</p>
<p align="center">
  Fig.1 Abstract architecture for interaction between the simulated teacher and the learning agent
</p>

For detailed instructions, please read the [assignment description](https://github.com/SeyedHamidreza/cognitive_robotics_ws/blob/main/first_assignment_cognitive_robotics.pdf).

## Requirements and Installation

you can simply use the provided bash script to install all necessary packages on your machine:

```bash
cd ~
sudo chmod +x setup_bashrc.sh
./setup_bashrc.sh
```

then, check your bashrc file: 

```bash
gedit .bashrc

```

The following lines should have been added at the end of your bashrc

```bash
#Add ROS cognitive_robotics_ws as workspace
source /opt/ros/melodic/setup.bash
export LD_LIBRARY_PATH=/opt/ros/melodic/lib:${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
export ROS_PACKAGE_PATH=$HOME/cognitive_robotics_ws:/opt/ros/melodic/include:${ROS_PACKAGE_PATH}

export ROBOT=sim
export PATH=~/bin:$PATH

export PATH=/usr/lib/python2.7/:$PATH
export LC_NUMERIC="en_US.UTF-8" 

#This line forces the system to use python2 by default since the ROS Melodic still works based on python2
alias python='/usr/bin/python2.7'

```


then, you need to install all the necessary packages and softwares, including, ROS melodic, TF, Keras, and Leveldb. 

```bash
cd ~
sudo chmod +x setup_all_required_packages.sh
./setup_all_required_packages.sh
```


TODO
