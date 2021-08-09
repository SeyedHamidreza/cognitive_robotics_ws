#### Representation of Tracked Object View

### msg: RTOV.msg
### Key: RTOV_<TrackID>_<ViewID>
### Topic name: rtov_<TrackID>

### Value:
Header header   ## ROS msg header
uint32 track_id ##Track ID given by Object Tracking
uint32 view_id  ##View ID given by Object Tracking
string[] sitov_keys  #list of spin image key (SITOV)

##Position/pose can be added
