# Fused forward-collision warning published by fusion_node
uint8 SAFE=0
uint8 CAUTION=1
uint8 BRAKE=2

std_msgs/Header header
uint8 level              # SAFE, CAUTION or BRAKE
float32 distance_m       # latest ultrasonic distance (NaN if stale)
bool pedestrian_detected # camera saw a pedestrian in the forward zone
bool ultrasonic_ok       # ultrasonic data is fresh
bool camera_ok           # camera detections are fresh
string reason            # human-readable explanation of the decision
