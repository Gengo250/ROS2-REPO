# Phase 2 storage

The independent Recorder subscribes to real `/odom` (`nav_msgs/msg/Odometry`) and
`/imu/data` (`sensor_msgs/msg/Imu`) using sensor-data QoS. ROS callbacks convert
messages and enqueue them without doing storage I/O. A separate process owns the
Mosaico SDK, Arrow transport, finalization and read-back verification. Neither
the Gateway nor Gazebo depends on Mosaico.

## Python environments

Use Python 3.12 with ROS 2 Jazzy. Keep the Gateway in `.venv` as documented in
[DEVELOPMENT.md](DEVELOPMENT.md). Do not install the SDK or NumPy 2 into that venv.
The Recorder uses a separate venv without system-site-packages; sourcing ROS makes
the Jazzy Python modules available, while the Recorder's dependencies remain local.

Verified setup, including machines without `ensurepip`:

```bash
cd ~/ros2_ws
# Gateway environment must already exist (see DEVELOPMENT.md).
python3 -m venv --without-pip .venv-recorder
env -u PYTHONPATH .venv/bin/python -m pip --python .venv-recorder install \
  pip -r src/digital_twin_recorder/requirements.txt pytest==8.4.2
env -u PYTHONPATH .venv-recorder/bin/python -m pip check
source /opt/ros/jazzy/setup.bash
source install/setup.bash
.venv-recorder/bin/python - <<'PY'
import rclpy, mosaicolabs, pyarrow, numpy, pydantic
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.serialization import serialize_message, deserialize_message
for cls in (Odometry, Imu):
    msg = cls()
    assert deserialize_message(serialize_message(msg), cls) == msg
print(pyarrow.__version__, numpy.__version__, pydantic.__version__)
PY
.venv-recorder/bin/python -m digital_twin_recorder.ros.recorder_node --help
```

Pins: mosaicolabs 0.7.0, pyarrow 26.0.0, numpy 2.4.6, pydantic 2.14.1,
PyYAML 6.0.1. PyYAML is needed by rclpy. The first environment was incomplete:
missing YAML and SDK transitive dependencies; temporarily exposing system packages
resolved imports but exposed an incompatible system SciPy. Installing dependencies
inside the isolated venv resolved this without touching ROS or the Gateway.

Run pip checks without the sourced workspace's PYTHONPATH: that overlay also
exposes Gateway package metadata even though its dependencies intentionally live
in the other venv. Recorder tests disable automatic pytest plugin discovery so
unrelated ROS launch-testing plugins are not imported into the storage venv.

## Infrastructure

Use exactly one deployment method. This validation uses the native implementation:
PostgreSQL 16.15 on `127.0.0.1:55432` and official `mosaicod` 0.7.0-rc on
`127.0.0.1:6726`, with SDK 0.7.0. The release archive's SHA-256 was checked.
The daemon reports its actual release-candidate version; it is not relabeled as GA.

```bash
cd ~/ros2_ws
# First installation only: downloads/extracts tools into .tools, no system install.
bash infrastructure/mosaico/bootstrap_native.sh
# Subsequent runs:
python3 infrastructure/mosaico/local_services.py start
.venv-recorder/bin/python - <<'PY'
from mosaicolabs import MosaicoClient
with MosaicoClient.connect(host='127.0.0.1', port=6726) as client:
    print(client.info().version)
    print(client.list_sequences())
PY
```

`client.version()` is not an API in the installed SDK. `client.info().version`
performs the actual server RPC. The source project is
[mosaico-labs/mosaico](https://github.com/mosaico-labs/mosaico); the locally installed
SDK and successful real round trips define the tested compatibility.

Persistent metadata lives in `.runtime/mosaico/postgres`; objects live under
`.runtime/mosaico/objects/telemetry`. The password file is local, ignored and mode
0600. Never copy it into evidence. Native shutdown targets the recorded executable
PID and private PostgreSQL data directory; it does not kill unrelated processes.
The daemon has up to 30 seconds to drain active streams on shutdown. If an
active Arrow Flight upload prevents exit, the manager logs a fallback and sends
SIGKILL only to the verified owned daemon PID, then waits for exit before stopping
PostgreSQL. Active recordings consequently fail; completed sequences remain
persistent and are verified after restart.

Docker Compose is an alternative, reviewed and configuration-validated but not
runtime-tested in this execution. It uses named `metadata`/`objects` volumes,
PostgreSQL health gating, loopback port publication and a digest-pinned daemon.
Supply `MOSAICO_DB_PASSWORD` through an untracked `.env`. Stop native services
before using Compose. Do not use `down -v`. The integration harness intentionally
uses native services and refuses occupied service ports.

## Record real telemetry

Terminal 1 (Gateway interpreter owns the ROS launch):

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
ros2 launch digital_twin_gateway digital_twin.launch.py scenario:=normal
```

Terminal 2:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic info /odom --verbose --no-daemon --spin-time 3
ros2 topic info /imu/data --verbose --no-daemon --spin-time 3
.venv-recorder/bin/python -m digital_twin_recorder.ros.recorder_node \
  --scenario normal --duration 20 --output recordings
```

Use the same `ROS_DOMAIN_ID`/discovery settings in both terminals. Duration is
wall time after storage becomes ready; `--duration 0` records until Ctrl+C.
The output is `recordings/<execution_id>/report.json`. An execution ID includes
UTC creation time and a UUID suffix. Reports retain source ROS timestamps,
reception wall timestamps, IDs, frames, poses, velocities, covariances, sample
counts, hashes, configuration hashes and stop reason.

Lifecycle: idle → connecting → recording → finalizing → completed. Completion
requires both topics, finalized SDK session, no ingestion notifications and a
read-back count/hash match. Failures are terminal: delayed worker events cannot
turn failed recordings into completed ones. Queue capacity defaults to 2048;
I/O heartbeat deadline is 15 s and finalization deadline 30 s. Failed workers are
joined, then terminated/killed if necessary. Partial sequences may remain for
diagnosis, but retrieval rejects them as incomplete. No automatic data deletion
occurs. A new recording is required after failure or simulation clock reset.

## Historical retrieval after restart

Stop the ROS launch with Ctrl+C first; wait for shutdown. Then:

```bash
cd ~/ros2_ws
python3 infrastructure/mosaico/local_services.py restart
source /opt/ros/jazzy/setup.bash
source install/setup.bash
.venv-recorder/bin/python -m digital_twin_recorder.storage.retrieval
# Set this to the execution_id printed by the recording command:
recording_id=YOUR_EXECUTION_ID
.venv-recorder/bin/python src/digital_twin_recorder/scripts/verify_persistence.py \
  --sequence "$recording_id" --expected "recordings/$recording_id/report.json" \
  --output /tmp/retrieved.json
python3 infrastructure/mosaico/local_services.py stop
```

The verifier streams the actual stored records, reconstructs each typed ontology,
checks identity, monotonic source times and reception timestamps, and compares
SHA-256 over all canonical fields against both the stored finalization summary
and the original source report. First/last records and storage bytes are retained
in its JSON output. Gazebo is not needed for retrieval.

## Automated validation and shutdown

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv-recorder/bin/python -m pytest \
  src/digital_twin_recorder/test -q
export PATH="$PWD/.tools/node_modules/.bin:$PATH"
npm --prefix web/digital-twin test
npm --prefix web/digital-twin run build
# Ports 8000, 5173, 6726 and 55432 must be free; stop owned services first.
python3 infrastructure/mosaico/local_services.py stop
python src/digital_twin_recorder/scripts/validate_phase2.py
```

The full harness creates a unique evidence directory. It starts native storage,
real headless Gazebo, Gateway, Recorder, Vite and Chromium; records during all
browser tests; measures WebSocket frequency; checks missing-IMU failure; stops
storage during a second recording; verifies ongoing telemetry and bounded worker
lifetime; tests unavailable startup; stops Gazebo; restarts both storage services;
retrieves and hashes the original data; rejects the partial outage sequence; and
stops owned services in `finally`. No fake publishers are used by this harness.
Unit tests use isolated test doubles for deterministic queue/deadline failures.

For manual operation: stop the Recorder first (wait for its final report), then
Ctrl+C in the ROS launch and Vite terminals, then run `local_services.py stop`.
Never delete `.runtime`, recordings or Docker volumes to perform a restart.
