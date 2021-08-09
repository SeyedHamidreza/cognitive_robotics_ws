#### Grasp Info

### msg: GTOV.msg
### Key: GTOV_<TrackID>_<ViewID>
### Topic name: GTOV_<TrackID>

### Value:
Header header   ## ROS msg header
uint32 track_id ##Track ID given by Object Tracking
uint32 view_id  ##View ID given by Object Tracking

## Spin_Image of the grasp point
float32[] spin_image 

## Spin_Image of the grasp point
float32[] spin_images_object

##Position position of the GraspPoint in Object
geometry_msgs/Vector3 graspPoint 

##Position and orientation of the object
geometry_msgs/PoseStamped pose_stamped ##The pose of the box frame

##Position of a robot's endefector 
geometry_msgs/PoseStamped arm_pose ##The pose of the box frame
geometry_msgs/Vector3 dimensions ##The dimensions of the box

##Finger Position 
uint32 fingers

##Recognition Result
string object_label  ##category label
float32 minimum_distance   ##Distance to category

