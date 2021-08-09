# contains a labeled bounding box that can be used to draw a marker

# namespace and id
string ns
int32 id

# marks the specified marker for deletion
bool delete

# a label for the bbox
string label

# color of the label
std_msgs/ColorRGBA label_color

# the bbox itself
BoundingBox bbox

# color of the bbox
std_msgs/ColorRGBA bbox_color

# How long the object should last before being automatically deleted.  0 means forever
duration lifetime
