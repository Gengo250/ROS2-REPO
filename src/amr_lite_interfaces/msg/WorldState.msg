# Complete world snapshot of the lite engine. header.stamp is simulation time, frame "world".
std_msgs/Header header
string session_id
uint32 epoch                 # increments when the simulation is reset
float64 sim_time
float64 real_time_factor     # measured over the last second of wall time
float64 target_real_time_factor
bool paused
uint64 overruns              # loop deadlines missed since start
string scene_sha256
EntityState[] entities
Obstacle[] obstacles
