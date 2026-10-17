# Phase 2 visualization

The Three.js viewer displays the existing warehouse and original AMR at their
configured initial poses. Gazebo remains responsible for physics, sensors and
collisions. Live odometry remains visible in the diagnostics beneath the viewer;
3D robot/person motion and localization are explicitly deferred to Phase 3.

## Geometry and coordinates

`GET /api/v1/scene?scenario=normal` reads the installed, hash-validated warehouse
compiler outputs. It uses the same SDF model includes, nested XYZ/RPY transforms,
station configuration and AMR dimensions as Gazebo. No second warehouse layout
is maintained. Invalid/stale compiler outputs fail explicitly.

The building is 30 × 20 × 5 m, on a 34 × 24 × 0.2 m floor. It contains 16 racks,
10 pillars and six stations. Renderer coordinates remain right-handed ROS world
coordinates: X east, Y north, Z up, metres. Fixed-axis RPY uses ZYX composition.
The glTF Y-up conversion is applied once to the original model; the enclosing
robot group receives the configured world spawn (-11, 0, 0.015, 0, 0, 0).
The GLB bounds are approximately 0.951 × 0.700 × 0.344 m, including protrusions.
Its configuration hash and bounds are checked when loading.

`WarehouseBuilder` batches repeated geometry with `InstancedMesh`. Shadows update
when the scene/structure changes. Camera controls provide overview, robot focus,
top-down, reset, left-drag orbit, right-drag pan and wheel zoom. ResizeObserver
updates the canvas and camera. Scenario changes dispose previous scene resources;
pagehide/HMR dispose controls, observers, geometry, materials, textures and WebGL.

The four previews select actual configured geometry:

| Scenario | Difference |
| --- | --- |
| normal | Normal configured warehouse and people |
| person_crossing | human_01 starts at (-8.7, -2.5), configured crossing heading |
| obstacle_in_aisle | Aisle pallet and warning cone |
| corridor_blocked | Full-width corridor barrier |

Preview selection does not change the running Gazebo scenario. All entities stay
at initial poses even while live diagnostic odometry changes.

## Run and validate

Use the environment and Gazebo/Gateway commands in [DEVELOPMENT.md](DEVELOPMENT.md).
In a separate terminal:

```bash
cd ~/ros2_ws
export PATH="$PWD/.tools/node_modules/.bin:$PATH"
npm --prefix web/digital-twin ci
npm --prefix web/digital-twin exec -- playwright install chromium
npm --prefix web/digital-twin run dev -- --strictPort
```

Open http://127.0.0.1:5173. The viewer requires WebGL2 and the Gateway scene API.
A missing GLB produces an explicit error while diagnostics remain usable.

```bash
npm --prefix web/digital-twin test
npm --prefix web/digital-twin run build
# With real Gateway, Gazebo and Vite already running, in a sourced ROS shell:
mkdir -p /tmp/phase2-browser
cd web/digital-twin
TWIN_SCENE_EVIDENCE_DIR=/tmp/phase2-browser TWIN_EVIDENCE_DIR=/tmp/phase2-browser npm run test:browser
```

Both variables are required for all four browser tests. The diagnostics test
commands a short real robot movement, stops it, and tests network recovery. This
is a Phase 1 regression check, not 3D animation. For owned startup and teardown,
prefer the full Phase 2 harness documented in [PHASE_2_STORAGE.md](PHASE_2_STORAGE.md).

Screenshots and performance measurements come from actual Chromium rendering.
`rendering.json` contains five FPS samples per scenario, their mean, load time,
draw calls, triangles, geometries, heap estimates, graphics backend and camera
control results. See [PHASE_2_VALIDATION.md](PHASE_2_VALIDATION.md) for measured
results and the software-rendering limitation on this machine. Stop Vite and the
ROS launch with Ctrl+C in their respective terminals.
