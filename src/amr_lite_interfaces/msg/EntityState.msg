# One simulated entity in the world frame (x east, y north, z up), at WorldState stamp.
string id
string type            # amr | human | forklift
float64 x
float64 y
float64 z              # floor level of the entity (mezzanine workers: 5.13)
float64 yaw
float64 vx             # world-frame linear velocity, m/s
float64 vy
float64 wz             # yaw rate, rad/s
float64 speed          # signed speed along the heading, m/s
string state           # amr: moving|stopped|blocked; human: walking|waiting|working|idle;
                       # forklift: moving|slowing|stopped|reversing
string activity        # human task (scan, pick, drop, inspect, kneel, signal, lookaround, idle) or empty
float32 activity_progress  # 0..1 within the current task
bool carrying
float32 look           # human head yaw relative to the body (perception target), rad
uint8 FIELD_FREE=0
uint8 FIELD_WARNING=1
uint8 FIELD_PROTECTIVE=2
uint8 field_state      # vehicles: speed-dependent safety field evaluation
float32 field_length   # protective field length ahead of the front edge, m
string reason          # short machine-readable reason for stopped/blocked/waiting states
