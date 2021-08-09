#CompleteRTOV msg

### Value:
Header header   ## ROS msg header
uint32 track_id ##Track ID given by Object Tracking
uint32 view_id  ##View ID given by Object Tracking
SITOV[] sitov
string ground_truth_name  ## Ground truth name

##Object pose 
geometry_msgs/PoseStamped pose_stamped ##The pose of the box frame
geometry_msgs/Vector3 dimensions ##The dimensions of the box

##Grasp point poses
##Position reperesents by a bounding box in space
##string  plane  ## later we need to add plan info too
geometry_msgs/PoseStamped[] grasp_poses # [x,y,z, quaternion]
uint32[] length
uint32[] width
string plane_with_max_entropy  ## plane with max entropy is used for multi-view grasp

bool is_key_view
bool object_lost

uint32 sign
