# header message
Header header

# calibrated joint offsets
string[] jointNames
float64[] jointOffsets

# transforms from the respective end effector to the marker
string[] chainNames
geometry_msgs/Transform[] endeffectorToMarker

# delta for transform from HeadPitch_link to CameraBottom_frame
geometry_msgs/Transform cameraTransform

# intrinsic camera parameters for the raw (distorted) images.
# K = (fx, fy, cx, cy)
float64[]  K