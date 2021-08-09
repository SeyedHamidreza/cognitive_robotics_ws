#### Recognition Result of Track Object View


### msg: RRTOV.msg
### Key: RRTOV_<TrackID>_<ViewID>
### Topic name: rrtov_<TrackID>
 
### Value:
Header header   ## ROS msg header
uint32 track_id ##Track ID given by Object Tracking
uint32 view_id  ##View ID given by Object Tracking
string recognition_result  ##Recognition Result
string ground_truth_name  ## Ground truth name
float32 minimum_distance  ## Minimum object categories distance
NOCD[] result ## array of normalized object category distance(NOCD)   
