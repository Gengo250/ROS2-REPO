# Discrete simulation event. seq increases monotonically within an engine session.
std_msgs/Header header
uint64 seq
string kind      # contact | protective_stop | blocked | released | obstacle_added | obstacle_removed | paused | resumed
string entity
string other
string detail
float64 x
float64 y
