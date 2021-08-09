# this message describes a single detected object (should always be used inside ObjectDetectionArray)
uint32 track_id ##Track ID given by Object Tracking

BoundingBox cluster_bbox # bbox of the cluster of points belonging to this object. See RM issue #820

# all possible types (with confidence scores) that this cluster could have
ObjectHypothesis[] hypotheses
