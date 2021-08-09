
sleep 2
 roslaunch race_perception_bringup visualize.launch use_pr2:=1 > /dev/null 2>&1 &
echo "rviz is running"
sleep 2
 roslaunch race_perception_bringup bringup_object_descriptor.launch use_standalone:=1 pdb_source:=/home/hamidreza/Desktop/RSS/pdb > /dev/null 2>&1 &
echo "perception is running"

