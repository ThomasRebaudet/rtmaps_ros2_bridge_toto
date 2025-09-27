std_msgs/Header header
uint8 state #Localization status: 0: Initialization | 1: Fix | 2: Error| 3: Discard input frame
string state_human_readable #State as a human readable string.
float64[16] pose #ENU frame - Pose as a 4x4 matrix, the upper-left 3x3 block being the normalized rotation matrix and the first three scalars of the last column the translation vector.
float64[36] pose_cov #Pose covariance matrix as a 6x6 diagonal matrix, the first three components being the covariance on the translation (m) and the last three the covariance on the rotation (rad).
float32[6] pose_std #pose standard deviation
float32[3] lla #Latitude, longitude and altitude in degrees as an array of length 3.
float32[6] velocity #velocity in x y z roll pitch yaw in map reference frame
float32[6] twist #Linear and angular velocities, concatenated in this order and expressed in the sensor reference frame.
float32 overlap #Ratio of overlap between the current frame and the static + local map.
