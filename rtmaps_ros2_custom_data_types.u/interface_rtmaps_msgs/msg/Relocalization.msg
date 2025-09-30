std_msgs/Header header
float64 timestamp
uint8  state
string state_human_readable #State as a human readable string.
float64[16] pose #ENU frame - Pose as a 4x4 matrix, the upper-left 3x3 block being the normalized rotation matrix and the first three scalars of the last column the translation vector.
float64[36] pose_cov #Pose covariance matrix as a 6x6 diagonal matrix, the first three components being the covariance on the translation (m) and the last three the covariance on the rotation (rad).
float32[6] pose_std #pose standard deviation
float32[3] lla #Latitude, longitude and altitude in degrees as an array of length 3.
float32[6] velocity #velocity in x y z roll pitch yaw in map reference frame
float32[6] twist #Linear and angular velocities, concatenated in this order and expressed in the sensor reference frame.
float32 overlap #Ratio of overlap between the current frame and the static + local map.
float32 overlap_static #Ratio of overlap between the current frame and the static map.
float32 processing_time #Processing time of the relocalization in seconds.
float32 distance_to_map
int32 num_pts_frame
int32 num_pts_icp
