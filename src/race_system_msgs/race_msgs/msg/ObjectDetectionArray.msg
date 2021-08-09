# this message describes the set of detected objects from one RGB (-D) camera frame / point cloud

# header.stamp and header.frame apply to all poses in the Detections
Header header

# the TF frame of the sensor (can be different from header.frame) 
string sensor_frame

# the list of detected objects
ObjectDetection[] objects
