#### Tracked Object View Info

### msg: TOVI.msg
### Key: TOVI_<TrackID>_<ViewID>
### Topic name: tovi_<TrackID>

### Value:
Header header   ## ROS msg header
uint32 track_id ##Track ID given by Object Tracking
uint32 view_id  ##View ID given by Object Tracking

##Position reperesents by a bounding box in space
geometry_msgs/PoseStamped pose_stamped ##The pose of the box frame
geometry_msgs/Vector3 dimensions ##The dimensions of the box

##Recognition Result
string object_label  ##category label
float32 minimum_distance   ##Distance to category

##Grasp point poses
##Position reperesents by a bounding box in space
##string  plane  ## later we need to add plan info too
geometry_msgs/PoseStamped[] grasp_poses # [x,y,z, quaternion]
uint32[] length
uint32[] width
string plane_with_max_entropy  ## plane with max entropy is used for multi-view grasp

string main_axis  ## current main axis
bool object_lost

int32 sign

