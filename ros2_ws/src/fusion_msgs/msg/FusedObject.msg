std_msgs/Header header
uint32 track_id

# Classification constants
uint8 UNKNOWN=0
uint8 CAR=1
uint8 PEDESTRIAN=2
uint8 CYCLIST=3
uint8 TRUCK=4
uint8 MOTORCYCLE=5

uint8 classification
float32 confidence

geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance velocity

float32 length
float32 width
float32 height

# Bitmask of active sensor sources (e.g. 0b001=camera, 0b010=lidar, 0b100=radar)
uint8 sensor_sources
