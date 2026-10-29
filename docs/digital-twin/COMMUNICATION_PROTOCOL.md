# Communication protocol — v1.0

Local default: `http://127.0.0.1:8000`. UTF-8 JSON, JSON Schema 2020-12.
Contracts: [TwinState](../../shared/schemas/twin-state.schema.json) and
[ConnectionStatus](../../shared/schemas/connection-status.schema.json).

| Interface | Semantics |
|---|---|
| GET `/api/v1/health` | Process responsiveness; `gateway: online`, `protocol_version: 1.0`. Does not assert ROS/simulation availability. |
| GET `/api/v1/status` | Latest connection snapshot plus current active socket count. |
| GET `/api/v1/state` | Latest published TwinState; repeated reads do not increment sequence. |
| WS `/ws/v1/twin?protocol_version=1.0` | Initial snapshot, then normalized snapshots at configured wall-clock rate. Version query optional, defaults to 1.0. |

No command endpoint. No raw ROS payloads, camera frames, point clouds or unlimited
queues. HTTP permits only explicitly allowed CORS origins. WebSocket checks Origin
separately. Non-browser diagnostic clients may omit Origin; this is not authentication.
An unsupported version or browser origin is denied before the WebSocket handshake.
Excess clients are denied; an application message is closed with code 1008. Shutdown
closes established sockets (1001, or Uvicorn's 1012 during server shutdown). A slow
send is timed out and disconnected. TLS and credentials are not implemented.

| TwinState field | Meaning |
|---|---|
| `protocol_version`, `message_type` | Exact literals `1.0`, `twin_state`. |
| `session_id` | UUID for this Gateway process; new on restart. |
| `sequence` | Strictly increasing per published snapshot in one session, starts at 1. Clients may skip intermediate values; initial snapshot can repeat an HTTP read. |
| `timestamp` | Timezone-aware UTC wall-clock snapshot creation time, ISO 8601. |
| `simulation_time` | Last `/clock` seconds, nullable, may pause/reset/backtrack. Check availability before treating it as current. |
| `world.id`, `world.frame_id` | World metadata loaded from the existing installed warehouse layout (explicit env overrides available). |
| `world.scenario`, `scenario_source` | Configured scenario and provenance (`launch_configuration`, `environment`, `unspecified`). Null when unspecified. Not telemetry verification. |
| `world.status` | `running` only while clock progress is fresh; otherwise `unknown` or `unavailable`. |
| `world.availability`, `status_source` | Four-state availability and literal `clock_progress`. No Gazebo service/process probe. |
| `robots` | Array of configured stable IDs with independent latest odometry. |
| `humans`, `obstacles`, `stations` | Reserved empty arrays; not tracking or environmental inventory. |
| `connection` | Same connection model as status endpoint, captured at snapshot time. |

| Robot field | Meaning |
|---|---|
| `id` | Stable configured ID, default `amr_01`. |
| `frame_id` | Original odometry header frame for pose; null before first sample. |
| `child_frame_id` | Original child frame for twist/velocity; null before first sample. |
| `pose_source` | Configured odometry topic, default `/odom`. |
| `pose.position` | x/y/z, metres. Null pose before first sample. |
| `pose.orientation` | Original quaternion x/y/z/w, unchanged. |
| `velocity.linear`, `velocity.angular` | x/y/z in m/s and rad/s in child frame. Null velocity before first sample. |
| `source_timestamp` | Original odometry header stamp converted to seconds; nullable. This is the source clock, not reception wall time. |
| `received_at` | UTC wall-clock reception time, nullable. |
| `age_seconds` | Monotonic wall time since last receipt, nullable; independent of `/clock`. |
| `status` | Freshness of the robot's own odometry. |

`ConnectionStatus` includes session/version/type, Gateway online status, ROS
initialization/executor status, simulation availability, latest wall-clock odometry
receipt across robots, socket count, configured Hz, measured Hz (inverse interval
between the last two publications), and per-topic receipt counts/timestamps/ages.
ROS online means the local executor is initialized and alive, not that every remote
publisher is available. `/tf_static` remains online after its first receipt for the
executor lifetime; static data are not expected to stream. Dynamic TF and joints
expire normally. Their contents are not forwarded or applied to the robot pose.

Availability defaults:

| State | Condition |
|---|---|
| `unknown` | No usable observation yet during the first 10 s of Gateway uptime. |
| `online` | Receipt/progress age under 2 s and ROS executor alive. |
| `stale` | Previously observed but age is 2–10 s. Retain last values, mark old. |
| `offline` | No observation for 10 s, or ROS executor stopped. Inferred unavailability, not proof of physical disconnection. |

All ages use a monotonic wall clock. Simulation availability requires two differing
clock samples. A paused clock becomes stale even if identical samples keep arriving.
World and odom are **not** equated; there is no map transform or global localization.
No visualization offset is applied in Phase 1.

Full measured examples, including null/missing data in tests, are schema validated.
See `evidence/*/initial-state.json` and `final-state.json` for actual runtime payloads.
Minimal health response:

```json
{"gateway":"online","protocol_version":"1.0"}
```

Version changes: update Python models, regenerate JSON Schemas and browser types,
then run schema drift checks and both test suites. v1.0 uses strict fields; adding
new nonempty reserved collections requires a coordinated version revision. Browsers
reject incompatible data, indicate disconnection and retry with bounded backoff.
A changed `session_id` resets sequence comparison. No replay/history is promised.

## Phase 5 route extension (independent, version 1.0)

Two read-only REST resources; WebSocket v1/v2 payloads are unchanged.

| Endpoint | Contract | Content |
|---|---|---|
| `GET /api/v1/routes/graph` | `route-graph.v1.schema.json` (`message_type: route_graph`) | Static warehouse graph in `world`: nodes, one entry per connection (with its directed edge ids), map-validation status, stations, and the verified world → map SE(2) registration with its id. Served from the installed `amr_route_planner` artifact; 503 when absent. Fetch once per `graph_version`. |
| `GET /api/v1/routes/status` | `route-status.v1.schema.json` (`message_type: route_status_snapshot`) | Last `/route_planner/status` message (`route_status`, map frame) with `availability`, `received_at`, `age_seconds`; `route: null` before any route. |

`/route_planner/status` is a latched (`reliable`, `transient_local`, depth 1)
`std_msgs/String` carrying the versioned JSON document. The Gateway bounds it to
256 KiB and validates it with a strict model (unknown fields, versions or states are
rejected). Route geometry is **map-frame**; a client must transform it with the
graph's registration and must not draw it when the registration id, graph version or
map version disagree with the graph it holds.
