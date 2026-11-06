# Digital Twin recorder

Independent ROS 2 telemetry persistence through Mosaico 0.7 (`mosaicolabs` SDK, `mosaicod`).
Lite runtime (default): odometry, IMU, engine world state (AMRs, workers, forklifts), simulation
events, obstacles and navigation status (`--profile core`), plus Nav2 plans, AMCL/static TF and both
LiDARs (`--profile extended`). Gazebo warehouse: `--simulation-backend gazebo`.

- Recorder: `python -m digital_twin_recorder.ros.recorder_node` (isolated `.venv-recorder`).
- History: `python -m digital_twin_recorder.storage.history {health,list,show,export,trajectory,verify}`.
- Live topic inventory: `scripts/inspect_topics.py`.

See the [storage guide](../../docs/digital-twin/PHASE_2_STORAGE.md) for environments, Compose
services, profiles, metadata, failure semantics, historical retrieval and validation.
