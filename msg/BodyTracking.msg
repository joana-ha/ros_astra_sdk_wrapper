std_msgs/Header header
bool valid_body_status

int32 body_id
int32 rfoot_tracking_status
int32 lfoot_tracking_status
int32 rknee_tracking_status
int32 lknee_tracking_status
int32 rhip_tracking_status
int32 lhip_tracking_status
int32 base_spine_tracking_status
int32 mid_spine_tracking_status
#int32 gesture

# Point32 position2D # x,y in camera frame, z distance from camera

#Position of interesting joints

geometry_msgs/Point32 joint_position_rfoot
geometry_msgs/Point32 joint_position_lfoot
geometry_msgs/Point32 joint_position_rknee
geometry_msgs/Point32 joint_position_lknee
geometry_msgs/Point32 joint_position_rhip
geometry_msgs/Point32 joint_position_lhip
geometry_msgs/Point32 joint_position_base_spine
geometry_msgs/Point32 joint_position_mid_spine


