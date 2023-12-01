# a unique ID of the track
uint32      id
# time stamp of the last prediction of this track
time        last_prediction
# time stamp of the last correction (either from detection or tracking) of this track
time        last_correction
# time stamp of the last detection associated with this track
time        last_detection
# confidence that the track is a valid target
float64     confidence
# true if this track is the currently selected "best" track
bool        selected

# points corresponding to the last track of the target
sensor_msgs/PointCloud2   points

# mean of the track's position estimate
geometry_msgs/Point       position
# mean of the track's velocity estimate
geometry_msgs/Vector3     velocity
# mean of the track's acceleration estimate
geometry_msgs/Vector3     acceleration
# uncertainty of the track's position, velocity and acceleration
float64[81]               covariance

# number of detections, associated with this track
uint32      n_detections
# number of point clusters, associated with this track
uint32      n_clusters
