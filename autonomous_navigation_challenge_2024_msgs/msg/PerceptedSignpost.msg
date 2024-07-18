# This message is used to send the road sign information to the robot

# This is the type of the road sign
int8 signpost_type
int8 SIGNPOST_NONE=-1
int8 SIGNPOST_FORWARD=0
int8 SIGNPOST_LEFT=1
int8 SIGNPOST_RIGHT=2
int8 SIGNPOST_BACKWARD=3
int8 SIGNPOST_STOP=4

# This is the distance from the robot to the road sign
float32 distance

# This is the bounding box object of the road sign in the image
vision_msgs/BoundingBox2D bounding_box
