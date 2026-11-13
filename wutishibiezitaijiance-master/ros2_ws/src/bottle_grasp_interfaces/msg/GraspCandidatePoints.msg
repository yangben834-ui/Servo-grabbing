# Candidate pose is expressed in header.frame_id (normally camera optical frame).
std_msgs/Header header
int32 track_id
int32 class_id
string class_name
float32 confidence
geometry_msgs/Pose pose
string grasp_mode
float32 quality
float32 axis_table_angle_deg
# Number of published object points (limited by publish_point_count).
uint32 point_count

# Point cloud of this object, expressed in header.frame_id.
# XYZ coordinates are float32 in meters; uses the candidate header.
geometry_msgs/Point32[] object_points
