# Development

Run commands from `~/ros2_ws` unless noted. The ROS workspace, model geometry,
warehouse generator and four scenarios remain the existing project. This guide
assumes Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic and existing robot dependencies.
No global pip installs, containers, authentication, Nav2 or SLAM setup is required.

## Dependencies and build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
# Normal path when python3-venv is available:
python3 -m venv --system-site-packages .venv
# If ensurepip is unavailable but Ubuntu python3-pip is already installed:
# python3 -m venv --without-pip --system-site-packages .venv
.venv/bin/python -m pip install -r src/digital_twin_gateway/requirements.txt
# Ensure the colcon entrypoint itself uses this interpreter, even when an equal
# colcon-core version is already visible through system-site-packages:
.venv/bin/python -m pip install --ignore-installed --no-deps colcon-core==0.20.1
source .venv/bin/activate
colcon build --symlink-install
source install/setup.bash
# Expected: #!/home/<user>/ros2_ws/.venv/bin/python
head -1 install/digital_twin_gateway/lib/digital_twin_gateway/gateway
```

The virtual environment reuses Jazzy's system Python 3.12 ROS extensions. Its
FastAPI/Pydantic dependencies are installed only in `.venv`. Ubuntu rosdep package
names in package.xml describe dependencies, but use the pinned requirements for
the tested Python versions; older distro Pydantic versions may be insufficient.
Rebuilding with `/usr/bin/colcon` can select system Python and lose API dependencies;
activate `.venv` first, or explicitly use `.venv/bin/python -m colcon`.

Vite 8 requires Node 20.19+ or 22.12+; this project requires Node >=22.12 and was
tested with 22.23.3. If the system has only Node 18, install a project-local Node:

```bash
cd ~/ros2_ws
npm install --prefix .tools --no-audit --no-fund node@22.23.3
export PATH="$PWD/.tools/node_modules/.bin:$PATH"
node --version
cd web/digital-twin
npm ci
npm run build
# Browser tests only:
npx playwright install chromium
```

`.venv`, `.tools`, node_modules, dist and runtime caches are ignored by Git.
`web/COLCON_IGNORE` keeps all web dependencies outside colcon discovery.
The root workspace keeps its original packages and adds digital_twin_gateway and
digital_twin_recorder. The Recorder uses its own Python environment; see
[PHASE_2_STORAGE.md](PHASE_2_STORAGE.md).

## Run

Prepare each ROS terminal:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
```

Combined headless simulation + Gateway (preferred):

```bash
ros2 launch digital_twin_gateway digital_twin.launch.py scenario:=normal
# Other selectable scenarios: person_crossing, obstacle_in_aisle, corridor_blocked
```

Or independently, in separate ROS terminals:

```bash
# Gateway only: works without publishers/Gazebo; pose remains null until received.
ros2 run digital_twin_gateway gateway
# Existing headless warehouse; do not ALSO start the combined launch:
ros2 launch amr_simulation warehouse.launch.py scenario:=normal gui:=false
```

Standalone Gateway scenario is unknown unless explicitly configured with
`TWIN_SCENARIO=normal`; this is metadata, not scene detection. Keep the same
`ROS_DOMAIN_ID` and discovery configuration across ROS participants. DDS is
independent of Gazebo partition. The existing warehouse launch creates or inherits
`GZ_PARTITION`; the new launch deliberately does not override it. Set an explicit
partition in the simulation shell only when attaching Gazebo diagnostic tools.

Frontend terminal (works without Gazebo or Gateway):

```bash
cd ~/ros2_ws
export PATH="$PWD/.tools/node_modules/.bin:$PATH"
cd web/digital-twin
# Optional: cp .env.example .env.local and adjust endpoints.
npm run dev
# Open http://127.0.0.1:5173
```

Keep the existing GUI for debugging:
`ros2 launch amr_simulation warehouse.launch.py scenario:=normal gui:=true`.
Do not run two simulations on the same ROS domain. The Digital Twin launch itself
always selects headless mode and does not start RViz.

## Inspect and verify

```bash
curl --fail http://127.0.0.1:8000/api/v1/health
curl --fail http://127.0.0.1:8000/api/v1/status
curl --fail http://127.0.0.1:8000/api/v1/state
ros2 topic list -t --no-daemon
ros2 topic echo /odom --once
ros2 topic echo /clock --once
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers '{}'
# ros2 control list_controllers is optional; ros2controlcli is absent on the
# validation machine, so the service above is the tested portable alternative.
.venv/bin/python - <<'PY'
import asyncio, json, websockets
async def inspect():
    async with websockets.connect('ws://127.0.0.1:8000/ws/v1/twin?protocol_version=1.0') as ws:
        for _ in range(3):
            state = json.loads(await ws.recv())
            print(state['sequence'], state['world']['status'], state['robots'])
asyncio.run(inspect())
PY
```

## Configuration

All Gateway settings use `TWIN_` environment variables, validated at startup.

| Variable suffix | Default / purpose |
|---|---|
| `HOST`, `PORT` | `127.0.0.1`, `8000` |
| `PUBLICATION_HZ` | `10`, range 0.1–60 Hz |
| `STALE_SECONDS`, `OFFLINE_SECONDS` | `2`, `10`; offline must exceed stale |
| `SEND_TIMEOUT`, `MAX_CLIENTS` | `2` s, `64` |
| `ALLOWED_ORIGINS` | JSON array: `["http://localhost:5173","http://127.0.0.1:5173"]` |
| `SCENARIO` | Unspecified outside composed launch |
| `WORLD_ID`, `WORLD_FRAME` | Existing installed layout's world name/frame |
| `ROBOTS` | JSON array with stable `id`, absolute `odom_topic`, `joint_states_topic` |
| `CLOCK_TOPIC`, `TF_TOPIC`, `TF_STATIC_TOPIC` | `/clock`, `/tf`, `/tf_static` |

Example telemetry configuration (does not spawn additional robots):

```bash
export TWIN_ROBOTS='[{"id":"amr_01","odom_topic":"/odom","joint_states_topic":"/joint_states"},{"id":"amr_02","odom_topic":"/amr_02/odom","joint_states_topic":"/amr_02/joint_states"}]'
export TWIN_PUBLICATION_HZ=10
ros2 run digital_twin_gateway gateway
```

Vite variables: `VITE_API_URL`, `VITE_WS_URL` (see `.env.example`). They are embedded
at build time; restart Vite after edits. Match the browser origin, including port,
in the Gateway allowlist. `npm run preview` defaults to port 4173: explicitly allow
that origin if testing a production bundle. The Phase 2 viewer requires WebGL2, the original AMR GLB and the Gateway scene
API. Diagnostics remain usable when the viewer cannot load.

## Automated validation

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
colcon test --packages-select digital_twin_gateway
colcon test-result --test-result-base build/digital_twin_gateway --verbose
colcon test
colcon test-result --verbose
python src/digital_twin_gateway/scripts/export_schemas.py --check
export PATH="$PWD/.tools/node_modules/.bin:$PATH"
npm --prefix web/digital-twin test
npm --prefix web/digital-twin run build
# Starts its own isolated simulation, Gateway, Vite and Chromium; commands a
# short forward motion, sends zero commands, and tears down its own processes:
python src/digital_twin_gateway/scripts/validate_integration.py
# New timestamped evidence directory each run; ports 8000 and 5173 must be free.
# For other scenarios, without browser/motion (telemetry + shutdown smoke test):
python src/digital_twin_gateway/scripts/validate_integration.py \
  --scenario person_crossing --skip-browser
```

The integration harness uses ROS domain 212 and a unique Gazebo partition.
Override test domain with `TWIN_TEST_ROS_DOMAIN_ID` if occupied. It verifies a real
Chromium page, not just socket JSON. Screenshots, ROS service results, actual
payloads, displacement, rate measurements, process commands and cleanup records
are saved under `docs/digital-twin/evidence/<UTC timestamp>/`. No previous simulation
evidence is overwritten. Unit tests contain test doubles; the integration path
uses only real Gazebo/ROS telemetry.

If running only the standalone browser suite, start Vite separately and run
`npm run test:browser`: real simulation/motion test is skipped unless
`TWIN_EVIDENCE_DIR` is supplied by the integration harness. Phase 2 warehouse
checks also require `TWIN_SCENE_EVIDENCE_DIR`; the Phase 2 harness sets both.
See VALIDATION.md and PHASE_2_VALIDATION.md for executed results and regression scope.

## Stop and deployment boundary

Use Ctrl+C in the launch terminal and wait for `Application shutdown complete`
and the Gazebo process exit. Use Ctrl+C in Vite's terminal. With standalone
processes, stop both Gateway and warehouse terminals. No browser command controls
the robot. Integration motion always sends zero TwistStamped commands in finally;
the unchanged controller's command timeout is an additional stop mechanism.
Do not use broad `pkill` commands that could kill unrelated ROS work.

This is a localhost development configuration, not production-ready. Remote
operation requires an explicit bind address, exact allowed origins, TLS/WSS via a
reverse proxy, authentication/authorization, network access controls and resource
limits appropriate to deployment. Never expose an unauthenticated robotics control
interface. Cameras/point clouds need a separate future transport; future commands
need a separate validated authorization boundary.

## Phase 2 validation

See [visualization](PHASE_2_VISUALIZATION.md), [storage and exact service commands](PHASE_2_STORAGE.md),
and [final validation evidence](PHASE_2_VALIDATION.md). Run the Phase 2 harness with
both browser test groups enabled; the Phase 1 harness alone does not validate storage.
