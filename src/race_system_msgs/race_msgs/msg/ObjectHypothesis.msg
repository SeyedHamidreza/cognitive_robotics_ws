# this message describes one single possible type with confidence score that an object could have (should always be used inside ObjectDetection msg)

# the object class
string type

# the object's pose (doesn't have to be identical to bbox centroid)
geometry_msgs/PoseStamped pose

# a measure of the confidence level in this detection result
float32 confidence

# bbox of the object (pose of centroid + size)
BoundingBox bbox
