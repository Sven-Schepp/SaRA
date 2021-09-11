Header header

# This contains an array of Points as defined by geometry_msgs
# They represent the joint positions in Cartesian (x y z) space.
geometry_msgs/Point[] positions

# They represent the joint velocities in Cartesian (x y z) space.
geometry_msgs/Point[] velocities

# The time between the last two frame transmissions
float64 delta_t